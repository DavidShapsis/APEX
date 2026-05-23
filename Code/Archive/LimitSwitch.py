from gpiozero import Button

class LimitSwitch:
    def __init__(self, pin: int):
        # We store the device internally
        self._device = Button(pin, pull_up=True, bounce_time=0.05)

    @property
    def state(self) -> bool:
        """Returns True if pressed, False if released."""
        return self._device.is_pressed