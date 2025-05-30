import time
import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2 import Preview

def preprocess_mask(mask):
    # Gürültü azaltmak için morfolojik işlemler
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    return mask

def find_and_draw(contour_mask, frame, color_name, draw_color, min_area=1500):
    contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected = False
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            cv2.putText(frame, f"{color_name} ({int(area)})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
            detected = True
    return detected

def report_seen_colors(seen_red, seen_green):
    if seen_red:
        print("Algılanan renk: KIRMIZI")
        return "kirmizi"
    if seen_green:
        print("Algılanan renk: YESIL")
        return "yesil"
    return None

def main():
    picam2 = Picamera2()
    picam2.start_preview(Preview.QTGL)
    picam2.start()
    time.sleep(2)

    while True:
        frame_rgb = picam2.capture_array()
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

        # Kırmızı maske
        lower_red1 = np.array([162,123,18])
        upper_red1 = np.array([179,223,118])

        mask_red =  cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red = preprocess_mask(mask_red)

        # Yeşil maske
        lower_green = np.array([85, 97, 7])
        upper_green = np.array([105, 198, 107])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_green = preprocess_mask(mask_green)

        output_rgb = frame_rgb.copy()

        # Renk algılama ve çizim
        seen_red = find_and_draw(mask_red, output_rgb, "KIRMIZI", (255, 0, 0), min_area=5000)
        seen_green = find_and_draw(mask_green, output_rgb, "YESIL", (0, 255, 0), min_area=5000)

        # Algılanan renkleri raporla
        report_seen_colors(seen_red, seen_green)

        # Görüntüyü göster
        output_bgr = cv2.cvtColor(output_rgb, cv2.COLOR_RGB2BGR)
        cv2.imshow("Frame", output_bgr)

        # ESC tuşuna basıldığında çık
        if cv2.waitKey(1) & 0xFF == 27:
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
