import cv2
import numpy as np
from picamera2 import Picamera2, Preview
import time
import json
import os

CONFIG_FILE = "hsv_config.json"
h_margin, s_margin, v_margin = 10, 50, 50

# Her renk için ayrı saklama alanı
config = {
    "GREEN": {"hsv_sel": None, "lower": None, "upper": None},
    "RED":   {"hsv_sel": None, "lower": None, "upper": None}
}
next_color = "RED"  # İlk tıklama RED'e ait olacak

def load_config():
    global config
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            cfg = json.load(f)
        for clr in ("GREEN","RED"):
            if clr in cfg:
                for key in ("hsv_sel","lower","upper"):
                    val = cfg[clr].get(key)
                    if val is not None:
                        config[clr][key] = np.array(val, dtype=int)
        print("Konfigürasyon yüklendi:", cfg)
    else:
        print("Konfigürasyon dosyası bulunamadı, tıklayınca kaydedilecek.")

def save_config():
    to_dump = {}
    for clr in ("GREEN","RED"):
        to_dump[clr] = {}
        for key in ("hsv_sel","lower","upper"):
            arr = config[clr][key]
            to_dump[clr][key] = arr.tolist() if isinstance(arr, np.ndarray) else None
    with open(CONFIG_FILE, "w") as f:
        json.dump(to_dump, f, indent=2)
    print("Konfigürasyon kaydedildi:", to_dump)

def on_mouse(event, x, y, flags, param):
    global config, next_color
    if event == cv2.EVENT_LBUTTONDOWN and param['frame'] is not None:
        frame = param['frame']
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_sel = hsv_frame[y, x]
        h,s,v = hsv_sel

        lower = np.array([
            max(h - h_margin, 0),
            max(s - s_margin, 0),
            max(v - v_margin, 0)
        ])
        upper = np.array([
            min(h + h_margin, 179),
            min(s + s_margin, 255),
            min(v + v_margin, 255)
        ])

        # Şu anki next_color moduna ata ve kaydet
        config[next_color]["hsv_sel"] = hsv_sel
        config[next_color]["lower"]  = lower
        config[next_color]["upper"]  = upper

        print(f"[{next_color}] Seçilen HSV: {tuple(hsv_sel)}")
        print(f"[{next_color}] Lower: {lower.tolist()}, Upper: {upper.tolist()}")
        save_config()

        # Mode'u değiştir (RED→GREEN, GREEN→RED)
        next_color = "GREEN" if next_color == "RED" else "RED"

def main():
    global next_color

    load_config()

    picam2 = Picamera2()
    picam2.start_preview(Preview.QTGL)
    picam2.start()
    time.sleep(2)

    cv2.namedWindow("Cam")
    params = {'frame': None}
    cv2.setMouseCallback("Cam", on_mouse, param=params)

    while True:
        rgb = picam2.capture_array()
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        params['frame'] = frame.copy()

        disp = frame.copy()
        h, w = disp.shape[:2]

        # Üstte bir bilgi: bir sonraki tıklama hangi renk için
        cv2.putText(disp, f"Next click: {next_color}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        # Eğer konfigürasyon varsa küçük önizleme
        for clr, color_bgr in [("GREEN",(0,255,0)), ("RED",(0,0,255))]:
            cfg = config[clr]
            if cfg["lower"] is not None and cfg["upper"] is not None:
                mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV),
                                   cfg["lower"], cfg["upper"])
                small = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                                   (w//4, h//4))
                x0 = w - small.shape[1] - 10
                y0 = 10 if clr=="GREEN" else 20 + h//4
                disp[y0:y0+small.shape[0], x0:x0+small.shape[1]] = small
                cv2.putText(disp, clr, (x0, y0-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

        cv2.imshow("Cam", disp)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
