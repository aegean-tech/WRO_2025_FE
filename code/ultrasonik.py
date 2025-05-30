#!/usr/bin/env python3
from gpiozero import DistanceSensor
from time import sleep

def main():
    # echo ve trigger pinlerini BCM olarak belirt
    sensor = DistanceSensor(echo=5, trigger=6, max_distance=2.0)
    try:
        print("Ölçümlere baslamak için Ctrl+C ile durdurabilirsiniz.\n")
        while True:
            # gpiozero DistanceSensor.distance → 0.0–1.0 aralığında, max_distance’a oranlı
            distance_m = sensor.distance * sensor.max_distance
            distance_cm = distance_m * 100
            print(f"Mesafe: {distance_cm:.1f} cm")
            sleep(1)
    except KeyboardInterrupt:
        print("\nÖlçüm sonlandırıldı.")
    finally:
        # gpiozero, çıkarken GPIO’ları otomatik temizliyor
        pass

if __name__ == "__main__":
    main()
