import time
import cv2
import numpy as np
import json
import os
from picamera2 import Picamera2, Preview
from motor import Motor
from servo import steer_right, steer_left, steer_center

CONFIG_FILE = "hsv_config.json"


def load_hsv_config():
    if not os.path.exists(CONFIG_FILE):
        raise FileNotFoundError(f"{CONFIG_FILE} bulunamadı.")
    with open(CONFIG_FILE, "r") as f:
        cfg = json.load(f)
    lower_red   = np.array(cfg["RED"]["lower"])
    upper_red   = np.array(cfg["RED"]["upper"])
    lower_green = np.array(cfg["GREEN"]["lower"])
    upper_green = np.array(cfg["GREEN"]["upper"])
    return lower_red, upper_red, lower_green, upper_green


def preprocess_mask(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    return mask


def find_and_draw(contour_mask, frame, color_name, min_area=1500):
    contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = frame.shape[:2]
    third = w / 3
    last_result = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        cx = x + bw / 2

        if color_name == "KIRMIZI":
            box_color = (0,255,0) if cx < third else (255,255,0)
        else:
            box_color = (0,255,0) if cx > 2*third else (255,255,0)

        cv2.rectangle(frame, (x, y), (x+bw, y+bh), box_color, 2)
        cv2.putText(frame, f"{color_name} ({int(area)})", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
        cv2.circle(frame, (int(cx), int(y+bh/2)), 5, box_color, -1)

        status = "SIKINTI YOK" if box_color == (0,255,0) else "GEÇİLMESİ LAZIM"
        last_result = (color_name, status)

    return last_result


def main():
    motor = Motor(l_en=27, r_en=18, rpwm=23, lpwm=22, pwm_frequency=100)
    current_status = None
    last_detected_color = None

    lower_red, upper_red, lower_green, upper_green = load_hsv_config()

    picam2 = Picamera2()
    picam2.start_preview(Preview.QTGL)
    picam2.start()
    time.sleep(2)

    while True:
        rgb = picam2.capture_array()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        mask_red   = preprocess_mask(cv2.inRange(hsv, lower_red,   upper_red))
        mask_green = preprocess_mask(cv2.inRange(hsv, lower_green, upper_green))

        output = rgb.copy()
        h, w = output.shape[:2]
        third = w / 3

        cv2.line(output, (int(third), 0), (int(third), h), (255,255,255), 2)
        cv2.line(output, (int(2*third), 0), (int(2*third), h), (255,255,255), 2)

        red_result   = find_and_draw(mask_red,   output, "KIRMIZI", min_area=5000)
        green_result = find_and_draw(mask_green, output, "YESIL",   min_area=5000)

        result = green_result or red_result
        if result:
            last_detected_color, current_status = result
            print(f"Son renk: {last_detected_color}, Durum: {current_status}")
        else:
            current_status = None

        cv2.imshow("Frame", cv2.cvtColor(output, cv2.COLOR_RGB2BGR))
        if cv2.waitKey(1) & 0xFF == 27:
            break

        # current_status None ise hiçbir tespit yok demektir
        # her frame başında önce ortal

        if current_status == "GEÇİLMESİ LAZIM":
            if last_detected_color == "YESIL":
                steer_left()
            elif last_detected_color == "KIRMIZI":
                steer_right()
        steer_center()

            

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
