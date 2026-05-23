from machine import Pin

class FSR:
    def __init__(self, pin_num: int):
        # We use Pin.IN with no internal pull-up because you have a physical 1k resistor
        self._pin = Pin(pin_num, Pin.IN)

    @property
    def state(self) -> bool:
        """Returns True if the foot is touching the ground (Signal is High)."""
        return self._pin.value() == 1

    # def on_touchdown(self, callback):
    #     """Triggered when the foot hits the floor."""
    #     self._device.when_pressed = callback

    # def on_liftoff(self, callback):
    #     """Triggered when the foot leaves the floor."""
    #     self._device.when_released = callback
    def on_touchdown(self, callback):
        self._pin.irq(trigger=Pin.IRQ_RISING, handler=lambda pin: callback())

    def on_liftoff(self, callback):
        self._pin.irq(trigger=Pin.IRQ_FALLING, handler=lambda pin: callback())