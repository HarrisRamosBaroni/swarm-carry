import RPi.GPIO as GPIO
import time

PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        print(GPIO.input(PIN))
        time.sleep(0.1)

finally:
    GPIO.cleanup()
