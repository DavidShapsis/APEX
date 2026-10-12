# BTS7960 current-sense ratio: I_IS = I_load / K_ILIS. This is the datasheet's
# typical value; the guaranteed spread is 3000-14000 at 5 A, so absolute amps
# are only indicative -- thresholds are picked from measured readings anyway.
K_ILIS = 8500
# Exponential smoothing per main-loop pass (~2-3 ms). The IS output is chopped
# by the 1 kHz PWM -- it only reports current while the high side conducts -- so
# a single sample is meaningless; this averages it into a usable level.
SMOOTHING = 0.2
# With the 680R on every IS line, even the 7 mA worst-case fault current reads
# ~3.0 V, i.e. ~3750 counts. Anything at the top of the scale is not a current
# at all: it is a disconnected or unpowered MCP3208 (floating DOUT reads 0xFFF).
INVALID_RAW = 4090


class CurrentSense:
    """One joint's motor current, from its BTS7960's R_IS and L_IS outputs.

    Each IS pin is a current source into `load_ohms` (everything from IS to
    ground), read on its own ADC channel through a divider that scales the IS
    voltage by `divider_ratio`. Only the conducting half-bridge reports at any
    moment: `channel_amps` keeps the two sides apart (R = RPWM / forward drive,
    L = LPWM / reverse drive) and `amps` is their sum, the motor current
    whichever way it is turning.

    `adc` is anything with read_raw(channel), `vref` and `FULL_SCALE` -- the
    leg's MCP3208.
    """

    def __init__(self, adc, channels, load_ohms, divider_ratio=1.0):
        self._adc = adc
        self._channels = tuple(channels)
        volts_per_count = adc.vref / adc.FULL_SCALE
        self._amps_per_count = volts_per_count * K_ILIS / (load_ohms * divider_ratio)
        self._offsets = [0] * len(self._channels)
        self.channel_amps = [0.0] * len(self._channels)
        self.amps = 0.0
        # False while any channel reads like a dead ADC rather than a current.
        self.valid = True

    def tare(self, samples=64):
        """Record the zero-load reading. Only call while this joint's PWM is 0."""
        for i, ch in enumerate(self._channels):
            total = 0
            for _ in range(samples):
                total += self._adc.read_raw(ch)
            self._offsets[i] = total // samples
            self.channel_amps[i] = 0.0
        self.amps = 0.0

    def update(self):
        valid = True
        total = 0.0
        for i, ch in enumerate(self._channels):
            raw = self._adc.read_raw(ch)
            if raw >= INVALID_RAW:
                valid = False
            amps = max(0, raw - self._offsets[i]) * self._amps_per_count
            self.channel_amps[i] += SMOOTHING * (amps - self.channel_amps[i])
            total += self.channel_amps[i]
        self.valid = valid
        self.amps = total
        return total
