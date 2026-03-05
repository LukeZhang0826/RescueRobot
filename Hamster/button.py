# button.py
import utime
from machine import Pin

class Button:
    def __init__(self, pin_num, pull=Pin.PULL_UP):
        self.pin = Pin(pin_num, Pin.IN, pull)

    def wait_for_press(self, debounce_ms=1000):
        # active LOW (pull-up), same behavior as your original
        while self.pin.value() == 1:
            utime.sleep_ms(10)
        while self.pin.value() == 0:
            utime.sleep_ms(10)
        utime.sleep_ms(debounce_ms)