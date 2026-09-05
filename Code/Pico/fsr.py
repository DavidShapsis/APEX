from machine import Pin

class FSR:
    def __init__(self, pin_num: int):
        # We use Pin.IN with no internal pull-up because you have a physical 1k resistor
        self._pin = Pin(pin_num, Pin.IN)

    @property
    def state(self) -> bool:
        """Returns True if the foot is touching the ground (Signal is High)."""
        return self._pin.value() == 1

    def on_edge(self, callback, rising=True, falling=True):
        """Register ONE handler for touchdown and/or liftoff.

        A Pin holds a single IRQ handler, so the previous separate
        on_touchdown()/on_liftoff() could not both be used -- registering the
        second silently replaced the first. They also passed a lambda, and
        MicroPython forbids allocation inside a hard IRQ, so the handler could
        fail at the worst possible moment. One registration, one bound method.

        `callback` is invoked as callback(is_touchdown: bool).
        """
        trigger = 0
        if rising:
            trigger |= Pin.IRQ_RISING
        if falling:
            trigger |= Pin.IRQ_FALLING
        self._callback = callback
        self._pin.irq(trigger=trigger, handler=self._on_irq)

    def _on_irq(self, pin):
        # Bound method, no allocation: safe in a hard IRQ.
        self._callback(pin.value() == 1)