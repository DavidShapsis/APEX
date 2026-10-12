import time


class TouchdownDetector:
    """Early foot contact during swing, inferred from joint motor current.

    Basic by design: it assumes the robot is walking a normal gait, upright. In
    that case a leg that should be in the air has nothing to push against, so a
    sustained current spike mid-swing is read as the foot touching down early --
    the same event the FSR used to report. It does not try to tell a touchdown
    from walking into something; both trip it.

    Two guards keep normal swing motion from tripping it:
      * `blank_ms` after liftoff is ignored. The leg is still pushing off and
        accelerating from rest there, and the PID saturates doing it.
      * the current must stay above threshold for `hold_ms`, so a one-pass
        spike (a direction reversal, ADC noise) does not count.

    `sensors` maps a joint name to its CurrentSense; `thresholds_amps` maps the
    same names to an amp threshold, or None to leave that joint out.
    """

    def __init__(self, sensors, thresholds_amps, hold_ms=10, blank_ms=40):
        self._joints = [(name, sensors[name], thresholds_amps[name])
                        for name in sensors
                        if thresholds_amps.get(name) is not None]
        self.hold_ms = hold_ms
        self.blank_ms = blank_ms
        self.reset()

    def reset(self):
        self._swing_start = None
        self._over_since = {}

    def update(self, in_swing, now_ms=None):
        """Call every pass. Returns the joint name that tripped, else None."""
        if not in_swing:
            self.reset()
            return None
        now = time.ticks_ms() if now_ms is None else now_ms
        if self._swing_start is None:
            self._swing_start = now
        if time.ticks_diff(now, self._swing_start) < self.blank_ms:
            return None
        for name, sensor, threshold in self._joints:
            # A dead ADC is not a touchdown; pico_main reports it separately.
            if sensor.valid and sensor.amps > threshold:
                since = self._over_since.get(name)
                if since is None:
                    self._over_since[name] = now
                elif time.ticks_diff(now, since) >= self.hold_ms:
                    return name
            else:
                self._over_since.pop(name, None)
        return None
