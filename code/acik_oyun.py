import time
from ultrasonik import setup_sensor, get_distance
from servo import steer_left, steer_right, steer_center
from motor import forward, stop, cleanup

# Ultrasonik pin tanımları (BCM)
FRONT_TRIG, FRONT_ECHO = 23, 24
LEFT_TRIG,  LEFT_ECHO  = 17, 27
RIGHT_TRIG, RIGHT_ECHO = 5,  6

# Ayarlar
FRONT_MIN_DIST = 30    # ön duvara yaklaşınca durma mesafesi (cm)
SIDE_BALANCE_MARGIN = 10   # sol-sağ dengesizlik toleransı (cm)
OPENING_THRESHOLD   = 200  # açılım görme eşiği (cm)

def main():
    # sensörleri kur
    setup_sensor(FRONT_TRIG, FRONT_ECHO)
    setup_sensor(LEFT_TRIG,  LEFT_ECHO)
    setup_sensor(RIGHT_TRIG, RIGHT_ECHO)

    try:
        while True: # kaç tane yapacağımız kararlaştırılacak
            # her sensörden ölçüm al
            front = get_distance(FRONT_TRIG, FRONT_ECHO)
            left  = get_distance(LEFT_TRIG,  LEFT_ECHO)
            right = get_distance(RIGHT_TRIG, RIGHT_ECHO)


            # 1) İki duvar arasında dengede kalma (sol-sağ eşitleme)
            delta = left - right
            if abs(delta) > SIDE_BALANCE_MARGIN:
                if delta > 0:
                    steer_right()   
                else:
                    steer_left()    
            else:
                steer_center()

            if front > FRONT_MIN_DIST and left < OPENING_THRESHOLD:
                forward()
            else:
                stop()
                # ön engel çok yakın: bekle ya da geriye çekilmek istersen burada ekleyebilirsin
                time.sleep(0.2)
                continue
            
            if front < FRONT_MIN_DIST and left > OPENING_THRESHOLD:
                steer_left()
                time.sleep(0.1)
                forward(0.3)
                steer_center()
            
            if front < FRONT_MIN_DIST and right > OPENING_THRESHOLD:
                steer_right()
                time.sleep(0.1)
                forward(0.3)
                steer_center()

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
        print("Program durdu.")

if __name__ == "__main__":
    main()
