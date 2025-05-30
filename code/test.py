import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
from motor import Motor

# ——————————————
# 1) Kalibrasyon Parametreleri
# ——————————————
KNOWN_DISTANCE       = 30.0   # cm
KNOWN_WIDTH          = 5.0    # cm
MEASURED_PIXEL_WIDTH = 200    # px
FOCAL_LENGTH         = (MEASURED_PIXEL_WIDTH * KNOWN_DISTANCE) / KNOWN_WIDTH

# Beyaz aralığı (HSV)
WHITE_LOWER = np.array([0, 0, 200])
WHITE_UPPER = np.array([180, 25, 255])

# Hedef mesafe aralığı ve tolerans (örnek: ~10 cm ±2 cm)
TARGET_DISTANCE      = 10.0
DISTANCE_TOLERANCE   = 2.0

# Servo ayarları
SERVO_PIN       = 17    # BCM pin
SERVO_FREQ      = 50
STEP_ANGLE      = 2.0   # Her adımda kaç derece hareket
CENTER_THRESH_PX = 20   # Piksel eşiği

# ——————————————
# 2) Servo Kontrol Sınıfı
# ——————————————
class ServoController:
    def __init__(self, pin, freq=SERVO_FREQ):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.pin    = pin
        self.freq   = freq
        self.min_dc = 2.5    # 0°
        self.max_dc = 12.5   # 180°
        self.angle  = 90.0   # orta pozisyon
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(self._angle_to_duty(self.angle))
        time.sleep(0.5)

    def _angle_to_duty(self, angle):
        return self.min_dc + (angle/180.0)*(self.max_dc - self.min_dc)

    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        self.angle = angle
        self.pwm.ChangeDutyCycle(self._angle_to_duty(angle))
        time.sleep(0.1)
        self.pwm.ChangeDutyCycle(0)

    def center(self):
        self.set_angle(90.0)

    def step_left(self):
        self.set_angle(self.angle - STEP_ANGLE)

    def step_right(self):
        self.set_angle(self.angle + STEP_ANGLE)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

# ——————————————
# 3) Yardımcı Fonksiyonlar
# ——————————————
def distance_to_camera(pixel_width):
    return (KNOWN_WIDTH * FOCAL_LENGTH) / pixel_width

def mask_white(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

def draw_crosshair(frame):
    h, w = frame.shape[:2]
    cx, cy = w//2, h//2
    cv2.line(frame, (cx-20, cy), (cx+20, cy), (0,255,255), 1)
    cv2.line(frame, (cx, cy-20), (cx, cy+20), (0,255,255), 1)
    return cx, cy

# ——————————————
# 4) Ana Döngü
# ——————————————
def main():
    servo = ServoController(SERVO_PIN)
    servo.center()

    # ? Motor �rne�ini olu�tur
    motor = Motor()
    motor.stop()

    cam = Picamera2()
    config = cam.create_preview_configuration(main={"format":"BGR888","size":(640,480)})
    cam.configure(config)
    cam.start()

    try:
        while True:
            frame = cam.capture_array()
            mask = mask_white(frame)
            cx_frame, cy_frame = draw_crosshair(frame)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            target = None
            for cnt in contours:
                if cv2.contourArea(cnt) < 500:
                    continue
                x,y,w,h = cv2.boundingRect(cnt)
                dist = distance_to_camera(w)
                if abs(dist - TARGET_DISTANCE) <= DISTANCE_TOLERANCE:
                    target = (x + w//2, y + h//2, dist)
                    break

            if target:
                tx, ty, td = target
                dx = tx - cx_frame

                # ? Servo ile yatay hizalama
                if dx > CENTER_THRESH_PX:
                    servo.step_right()
                elif dx < -CENTER_THRESH_PX:
                    servo.step_left()

                # ? Mesafeye g�re motor kontrol�
                if td > TARGET_DISTANCE + DISTANCE_TOLERANCE:
                    motor.forward(speed=60)   # obje uzakta, yakla�
                elif td < TARGET_DISTANCE - DISTANCE_TOLERANCE:
                    motor.backward(speed=40)  # obje �ok yak�nsa geri
                else:
                    motor.stop()              # do�ru mesafedeyse dur

                # G�rsel geri bildirim
                cv2.circle(frame, (tx, ty), 5, (0,255,0), -1)
                cv2.putText(frame, f"{td:.1f}cm", (tx+10, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            else:
                motor.stop()  # hedef yoksa dur

            cv2.imshow("Mask White", mask)
            cv2.imshow("Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.stop()
        servo.cleanup()
        motor.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
