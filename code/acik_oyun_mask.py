import cv2
import numpy as np
from picamera2 import Picamera2

# ��������������
# 1) Kalibrasyon Parametreleri
# ��������������
# Bilinen mesafede (cm) boyutu KNOWN_WIDTH kadar olan objenin piksel cinsinden geni�li�ini �l��n:
KNOWN_DISTANCE = 30.0      # cm (kalibrasyon objesinin kameraya uzakl���)
KNOWN_WIDTH    = 5.0       # cm (objenin ger�ek geni�li�i)
MEASURED_PIXEL_WIDTH = 200 # px (o mesafede �ekti�iniz objenin g�r�nt�deki geni�li�i)

# Odak uzakl��� (fokal uzunluk) hesaplan�yor:
FOCAL_LENGTH = (MEASURED_PIXEL_WIDTH * KNOWN_DISTANCE) / KNOWN_WIDTH

# ��������������
# 2) HSV E�ik De�erleri
# ��������������
# Pembe aral��� (�rnek; gerekirse kameran�za g�re ayarlay�n)
PINK_LOWER = np.array([140, 50, 50])
PINK_UPPER = np.array([170, 255, 255])

# Beyaz aral���
WHITE_LOWER = np.array([0,   0, 200])
WHITE_UPPER = np.array([180, 25, 255])

# ��������������
# 3) Yard�mc� Fonksiyonlar
# ��������������
def distance_to_camera(pixel_width):
    """
    Piksel geni�li�inden (w) cm cinsinden uzakl�k d�ner.
    """
    return (KNOWN_WIDTH * FOCAL_LENGTH) / float(pixel_width)

def mask_colors(frame):
    """
    BGR frame � HSV � pembe & beyaz maskeleri olu�turur.
    Beyaz maskeye biraz morfolojik temizlik uygular.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_pink  = cv2.inRange(hsv, PINK_LOWER, PINK_UPPER)
    mask_white = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    # Beyaz maskeyi temizleyelim
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN,  kernel, iterations=2)
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask_pink, mask_white

# ��������������
# 4) Ana D�ng�
# ��������������
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
