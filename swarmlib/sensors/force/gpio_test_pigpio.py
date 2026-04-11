import time
import pigpio

PIN = 24

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("Cannot connect to pigpiod")

pi.set_pull_up_down(PIN, pigpio.PUD_DOWN)

try:
    while True:
        level = pi.read(PIN)
        print(level)
        time.sleep(0.2)

finally:
    pi.stop()
