import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
from motor import Motor  # Your H-bridge motor class

# ——————————————
# 1) Kalibrasyon & HSV
# ——————————————
KNOWN_DISTANCE       = 30.0   # cm
KNOWN_WIDTH          = 5.0    # cm
MEASURED_PIXEL_WIDTH = 200    # px
FOCAL_LENGTH         = (MEASURED_PIXEL_WIDTH * KNOWN_DISTANCE) / KNOWN_WIDTH

WHITE_LOWER = np.array([0,   0, 200])
WHITE_UPPER = np.array([180, 25, 255])

# Mavi aralığı (HSV)
BLUE_LOWER  = np.array([100, 150, 50])
BLUE_UPPER  = np.array([140, 255, 255])

TARGET_DISTANCE    = 10.0   # cm
DISTANCE_TOLERANCE = 2.0    # cm tolerans

SERVO_PIN        = 17
STEP_ANGLE       = 2.0
CENTER_THRESH_PX = 20

NORMAL_SPEED = 100

# ——————————————
# 2) Servo Controller
# ——————————————
class ServoController:
    def __init__(self, pin, freq=50):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.pin    = pin
        self.min_dc = 2.5
        self.max_dc = 12.5
        self.angle  = 90.0
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, freq)
        self.pwm.start(self._duty(self.angle))
        time.sleep(0.5)

    def _duty(self, angle):
        return self.min_dc + (angle/180.0)*(self.max_dc - self.min_dc)

    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        self.angle = angle
        self.pwm.ChangeDutyCycle(self._duty(angle))
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

# ——————————————
# 3) Yardımcı Fonksiyonlar
# ——————————————
def distance_to_camera(w):
    return (KNOWN_WIDTH * FOCAL_LENGTH) / w

def mask_color(hsv, lower, upper):
    m = cv2.inRange(hsv, lower, upper)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k, iterations=2)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
    return m

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
    motor = Motor()

    cam = Picamera2()
    cfg = cam.create_preview_configuration(main={"format":"BGR888","size":(640,480)})
    cam.configure(cfg)
    cam.start()

    cv2.namedWindow("Mask White", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mask Blue",  cv2.WINDOW_NORMAL)
    cv2.namedWindow("Detection",  cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = cam.capture_array()
            hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_w = mask_color(hsv, WHITE_LOWER, WHITE_UPPER)
            mask_b = mask_color(hsv, BLUE_LOWER,  BLUE_UPPER)
            cx_f, cy_f = draw_crosshair(frame)

            # Öncelikle MAVİ kontur ara
            blue_cnts, _ = cv2.findContours(mask_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blue_detected = any(cv2.contourArea(c) > 500 for c in blue_cnts)

            if blue_detected:
                # Mavi varsa servo sola, motor ileri
                servo.step_left()
                motor.forward(NORMAL_SPEED)
            else:
                # Mavi yok => beyaz takibe devam
                target = None
                white_cnts, _ = cv2.findContours(mask_w, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in white_cnts:
                    if cv2.contourArea(c) < 500: continue
                    x,y,w,h = cv2.boundingRect(c)
                    d = distance_to_camera(w)
                    if abs(d - TARGET_DISTANCE) <= DISTANCE_TOLERANCE:
                        target = (x+w//2, y+h//2, d)
                        break

                if target:
                    tx, ty, td = target
                    cv2.circle(frame, (tx, ty), 5, (0,255,0), -1)
                    cv2.putText(frame, f"{td:.1f}cm", (tx+10, ty),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    dx = tx - cx_f
                    if dx > CENTER_THRESH_PX:
                        servo.step_right()
                    elif dx < -CENTER_THRESH_PX:
                        servo.step_left()
                    motor.forward(NORMAL_SPEED)
                else:
                    motor.stop()

            # Görüntüleri göster
            cv2.imshow("Mask White", mask_w)
            cv2.imshow("Mask Blue",  mask_b)
            cv2.imshow("Detection",  frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()
        servo.cleanup()
        motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
