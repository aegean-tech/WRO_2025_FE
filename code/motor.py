import RPi.GPIO as GPIO
import time

class Motor:
    def __init__(self, l_en=27, r_en=18, rpwm=23, lpwm=22, pwm_frequency=100):
        # Set up GPIO mode and disable warnings
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.rpwm_pin = rpwm
        self.lpwm_pin = lpwm
        self.l_en_pin = l_en
        self.r_en_pin = r_en

        # Set all the pins as outputs
        GPIO.setup(self.rpwm_pin, GPIO.OUT)
        GPIO.setup(self.lpwm_pin, GPIO.OUT)
        GPIO.setup(self.l_en_pin, GPIO.OUT)
        GPIO.setup(self.r_en_pin, GPIO.OUT)

        # Enable both motor directions on the H-bridge
        GPIO.output(self.r_en_pin, True)
        GPIO.output(self.l_en_pin, True)

        # Initialize PWM on RPWM and LPWM pins with the specified frequency
        self.rpwm = GPIO.PWM(self.rpwm_pin, pwm_frequency)
        self.lpwm = GPIO.PWM(self.lpwm_pin, pwm_frequency)

        # Start PWM with 0 duty cycle
        self.rpwm.start(0)
        self.lpwm.start(0)

    def forward(self, speed=100):
        """İleri sürüş: right PWM = speed, left PWM = 0"""
        self.rpwm.ChangeDutyCycle(0)
        self.lpwm.ChangeDutyCycle(speed)

    def backward(self, speed=100):
        """Geri sürüş: left PWM = speed, right PWM = 0"""
        self.rpwm.ChangeDutyCycle(speed)
        self.lpwm.ChangeDutyCycle(0)

    def stop(self):
        """Motoru durdur"""
        self.rpwm.ChangeDutyCycle(0)
        self.lpwm.ChangeDutyCycle(0)

    def cleanup(self):
        """PWM’i durdur ve GPIO konfigürasyonunu temizle"""
        self.rpwm.stop()
        self.lpwm.stop()
        GPIO.cleanup()


def main():
    # Motor sürücüsü pinlerini varsayılan değerlerle başlat
    motor = Motor(l_en=27, r_en=18, rpwm=23, lpwm=22, pwm_frequency=100)
    motor.forward(100)
    try:
        print("Motor ileri")
        motor.forward(80)     # %80 hızla ileri
        time.sleep(3)         # 3 saniye bekle

        print("Motor durduruluyor")
        motor.stop()
        time.sleep(1)

        print("Motor geri")
        motor.backward(80)    # %80 hızla geri
        time.sleep(3)

        print("Motor durduruluyor")
        motor.stop()
        time.sleep(1)

    finally:
        print("GPIO temizleniyor")
        motor.cleanup()       # PWM ve GPIO’yu sıfırla

if __name__ == "__main__":
    main()
