import RPi.GPIO as GPIO
import time

servo_pin = 13  # PWM destekli GPIO pin (örnek: 18)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(servo_pin, GPIO.OUT)

# 50 Hz PWM başlat (standart servo frekansı)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)  # 0° için ~2, 180° için ~12 duty cycle
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

def steer_right():
    set_angle(180)

def steer_left():
    set_angle(0)

def steer_center():
    set_angle(90)


while True:
    steer_right()