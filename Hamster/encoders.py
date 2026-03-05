# encoders.py
import machine
from machine import Pin

QDEC_TABLE = (
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
)

class DualEncoder:
    """
    Quadrature x4 decoding for two encoders using shared ISR logic.
    Provides atomic read/reset for safe control loops.
    """
    def __init__(self, ea_a_pin, ea_b_pin, eb_a_pin, eb_b_pin, enc_inv_a=-1, enc_inv_b=-1):
        self.EA_A = Pin(ea_a_pin, Pin.IN, Pin.PULL_UP)
        self.EA_B = Pin(ea_b_pin, Pin.IN, Pin.PULL_UP)
        self.EB_A = Pin(eb_a_pin, Pin.IN, Pin.PULL_UP)
        self.EB_B = Pin(eb_b_pin, Pin.IN, Pin.PULL_UP)

        self.enc_inv_a = enc_inv_a
        self.enc_inv_b = enc_inv_b

        self.encA = 0
        self.encB = 0
        self.oldA = (self.EA_A.value() << 1) | self.EA_B.value()
        self.oldB = (self.EB_A.value() << 1) | self.EB_B.value()

        # Register interrupts
        self.EA_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irqA)
        self.EA_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irqA)
        self.EB_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irqB)
        self.EB_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irqB)

    def _irqA(self, pin):
        new = (self.EA_A.value() << 1) | self.EA_B.value()
        step = QDEC_TABLE[(self.oldA << 2) | new]
        self.encA += self.enc_inv_a * step
        self.oldA = new

    def _irqB(self, pin):
        new = (self.EB_A.value() << 1) | self.EB_B.value()
        step = QDEC_TABLE[(self.oldB << 2) | new]
        self.encB += self.enc_inv_b * step
        self.oldB = new

    def reset(self):
        irq_state = machine.disable_irq()
        self.encA = 0
        self.encB = 0
        machine.enable_irq(irq_state)

    def read_counts(self):
        irq_state = machine.disable_irq()
        a = self.encA
        b = self.encB
        machine.enable_irq(irq_state)
        return a, b