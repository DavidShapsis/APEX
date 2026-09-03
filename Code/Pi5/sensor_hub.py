"""
sensor_hub.py -- keeps blocking sensor I/O off the 100 Hz control loop.

pi5_main's control loop used to call imu.update(), compass.get_heading() and
the INA219 reads inline, every iteration. Those are synchronous I2C
transactions: on a healthy bus they cost a few ms, but on a NAKing or wedged
bus the kernel blocks for its I2C timeout -- and while it blocks, steering
resends, the IMU reflex and the avoidance stride command all wait with it.

SensorHub runs one poller thread per sensor. Each thread owns its hardware
object exclusively, reads at its own rate, and publishes the result into a
lock-protected snapshot. The control loop reads the snapshot -- a dict copy,
never hardware -- and applies a staleness cutoff, so a dead or hung sensor
degrades to "use the safe default" (flat attitude, hold heading, skip the
battery check) instead of stalling the gait.

Two independent failure signals:

  * snapshot freshness  -- the poller ran but the reading was bad or old
    (imu.update() returned None, or the last good sample is > max_age old).
  * thread_alive()      -- the poller thread itself has not completed a loop
    within `timeout`, i.e. it is blocked inside a hardware call right now.

GPSReader.update() is already non-blocking (it only drains the UART buffer),
so it rides along on the nav poller purely to keep all sensor I/O in one place.
"""

import threading
import time

# Per-sensor default poll rates (Hz). The IMU feeds gait levelling, which only
# rebuilds the gait on a > 1.5 deg attitude change, so 50 Hz is already
# generous; the QMC5883L runs at 200 Hz ODR internally so 10 Hz here always
# gets a fresh sample, and the GPS emits at 1 Hz;
# the INA219 battery check runs once a second in the loop.
IMU_HZ = 50.0
NAV_HZ = 10.0
POWER_HZ = 1.0

# Default staleness cutoffs (seconds) used by the *_snapshot() helpers.
IMU_MAX_AGE = 0.15      # ~7 missed 50 Hz samples
NAV_MAX_AGE = 1.0       # heading; GPS fix is checked separately by the caller
POWER_MAX_AGE = 3.0


class SensorHub:
    def __init__(self, imu=None, gps=None, compass=None, power=None,
                 imu_hz=IMU_HZ, nav_hz=NAV_HZ, power_hz=POWER_HZ,
                 logger=None):
        self.imu = imu
        self.gps = gps
        self.compass = compass
        self.power = power
        self._log = logger or (lambda msg: print(f"[SENSOR] {msg}"))

        self._lock = threading.Lock()
        self._imu = {"roll": 0.0, "pitch": 0.0, "ok": False, "t": 0.0}
        self._nav = {"lat": 0.0, "lon": 0.0, "has_fix": False,
                     "satellites": 0, "heading": None, "t": 0.0,
                     # "t" is bumped every poll (GPS drain succeeds even with no
                     # fix); "heading_t" only on a *good* compass read and
                     # "fix_t" only on a positional GGA, so a compass or GPS
                     # that starts failing mid-run is detectable even though the
                     # shared poller keeps ticking.
                     "heading_t": 0.0, "fix_t": 0.0}
        self._power = {"voltage": None, "current": None, "t": 0.0}

        # Heartbeat: monotonic time each poller last *finished* an iteration.
        # Frozen => that thread is blocked inside a hardware call.
        self._beat = {"imu": 0.0, "nav": 0.0, "power": 0.0}

        self._running = False
        self._threads = []
        self._specs = []
        if imu is not None:
            self._specs.append(("imu", self._imu_poll, imu_hz))
        if gps is not None or compass is not None:
            self._specs.append(("nav", self._nav_poll, nav_hz))
        if power is not None:
            self._specs.append(("power", self._power_poll, power_hz))

    # -- lifecycle --------------------------------------------------------

    def start(self):
        if self._running:
            return
        self._running = True
        now = time.monotonic()
        for name, _fn, _hz in self._specs:
            self._beat[name] = now
        for name, fn, hz in self._specs:
            t = threading.Thread(target=self._run, args=(name, fn, hz),
                                 name=f"sensor-{name}", daemon=True)
            t.start()
            self._threads.append(t)
        self._log(f"pollers started: {', '.join(n for n, _, _ in self._specs)}")

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads = []

    # -- poller driver --------------------------------------------------

    def _run(self, name, poll_fn, hz):
        period = 1.0 / hz
        next_t = time.monotonic()
        fails = 0
        last_log = 0.0
        while self._running:
            try:
                poll_fn()
                if fails:
                    self._log(f"{name}: recovered after {fails} failed poll(s)")
                fails = 0
            except Exception as e:
                fails += 1
                now = time.monotonic()
                if now - last_log > 5.0:
                    self._log(f"{name}: poll failed ({fails}x): {e}")
                    last_log = now

            self._beat[name] = time.monotonic()

            next_t += period
            now = time.monotonic()
            if next_t < now - period:      # fell badly behind; resync
                next_t = now + period
            time.sleep(max(0.0, next_t - now))

    # -- individual polls (run on their own thread only) ---------------

    def _imu_poll(self):
        data = self.imu.update()
        with self._lock:
            if data is not None:
                self._imu["roll"] = self.imu.get_roll()
                self._imu["pitch"] = self.imu.get_pitch()
                self._imu["ok"] = True
                self._imu["t"] = time.monotonic()
            else:
                self._imu["ok"] = False

    def _nav_poll(self):
        if self.gps is not None:
            self.gps.update()
        # Tilt-compensate the compass with the freshest attitude the IMU poller
        # has published. Stale/absent IMU -> 0,0 -> plain flat-mount heading.
        with self._lock:
            roll = self._imu["roll"] if self._imu["ok"] else 0.0
            pitch = self._imu["pitch"] if self._imu["ok"] else 0.0
        heading = (self.compass.get_heading(roll_deg=roll, pitch_deg=pitch)
                   if self.compass is not None else None)
        now = time.monotonic()
        with self._lock:
            if self.gps is not None:
                self._nav["lat"] = self.gps.lat
                self._nav["lon"] = self.gps.lon
                self._nav["has_fix"] = self.gps.has_fix
                self._nav["satellites"] = self.gps.satellites
                # GPSReader.fix_t is time.monotonic() (our clock) of its last
                # positional GGA; carried through so the caller can spot a GPS
                # that has gone silent with has_fix still latched True.
                self._nav["fix_t"] = self.gps.fix_t
            # Keep the last good heading if the read failed -- a wedged compass
            # should not yank the estimate to 0 deg (due north) -- but only
            # advance heading_t on a real reading so the caller can tell.
            if heading is not None:
                self._nav["heading"] = heading
                self._nav["heading_t"] = now
            self._nav["t"] = now

    def _power_poll(self):
        v = self.power.get_voltage()
        c = self.power.get_current()
        with self._lock:
            self._power["voltage"] = v
            self._power["current"] = c
            self._power["t"] = time.monotonic()

    # -- snapshots (safe to call at any rate from any one reader) ------

    def imu_snapshot(self, max_age=IMU_MAX_AGE):
        """{'roll','pitch'} in degrees, or None if the last good read is
        missing or older than max_age."""
        with self._lock:
            s = dict(self._imu)
        if not s["ok"] or (time.monotonic() - s["t"]) > max_age:
            return None
        return s

    def nav_snapshot(self):
        """Always returns a dict: lat, lon, has_fix, satellites, heading (may be
        None), t (monotonic of last poll), age (seconds since last poll),
        heading_age (seconds since the last *good* compass read), fix_age
        (seconds since the last positional GPS fix). heading_age / fix_age are
        +inf until the first good read."""
        with self._lock:
            s = dict(self._nav)
        now = time.monotonic()
        s["age"] = now - s["t"]
        s["heading_age"] = now - s["heading_t"] if s["heading_t"] else float("inf")
        s["fix_age"] = now - s["fix_t"] if s["fix_t"] else float("inf")
        return s

    def power_snapshot(self, max_age=POWER_MAX_AGE):
        """{'voltage','current'}, or None if never read or older than max_age."""
        with self._lock:
            s = dict(self._power)
        if s["voltage"] is None or (time.monotonic() - s["t"]) > max_age:
            return None
        return s

    # -- watchdog -----------------------------------------------------

    def thread_alive(self, name, timeout=1.0):
        """False if that poller has not finished a loop within `timeout` --
        i.e. it is blocked inside a hardware call."""
        return (time.monotonic() - self._beat.get(name, 0.0)) < timeout

    def health(self, timeout=1.0):
        """{name: bool} liveness for every running poller."""
        return {name: self.thread_alive(name, timeout)
                for name, _, _ in self._specs}


# ---------------------------------------------------------------------------
# Stand-alone self-test with fake sensors -- `python sensor_hub.py`
# ---------------------------------------------------------------------------

def _selftest():
    import math

    class FakeIMU:
        def __init__(self):
            self.n = 0

        def update(self):
            self.n += 1
            # Every 20th read fails, like a real dropped BNO08x report.
            return None if self.n % 20 == 0 else {"roll": 0.0}

        def get_roll(self):
            return 3.0 * math.sin(self.n / 10.0)

        def get_pitch(self):
            return -2.0

    class FakeGPS:
        lat, lon, has_fix, satellites = 41.05, -74.14, True, 9

        def __init__(self):
            self.fix_t = 0.0

        def update(self):
            self.fix_t = time.monotonic()
            return True

    class FakeCompass:
        def __init__(self):
            self.n = 0

        def get_heading(self, roll_deg=0.0, pitch_deg=0.0):
            self.n += 1
            return None if self.n % 15 == 0 else (self.n * 2.0) % 360.0

    class FakePower:
        def get_voltage(self):
            return 7.9

        def get_current(self):
            return 850.0

    class HangingIMU(FakeIMU):
        def update(self):
            time.sleep(5.0)      # simulate a wedged I2C bus

    ok = True
    hub = SensorHub(imu=FakeIMU(), gps=FakeGPS(), compass=FakeCompass(),
                    power=FakePower())
    hub.start()
    time.sleep(0.5)

    imu_s = hub.imu_snapshot()
    print("imu_snapshot     :", imu_s)
    ok &= imu_s is not None and abs(imu_s["pitch"] + 2.0) < 1e-9

    nav_s = hub.nav_snapshot()
    print("nav_snapshot     :", nav_s)
    ok &= nav_s["has_fix"] and nav_s["heading"] is not None and nav_s["age"] < 0.5
    ok &= nav_s["heading_age"] < 0.5      # a good compass read happened recently
    ok &= nav_s["fix_age"] < 0.5          # a positional GPS fix happened recently

    pow_s = hub.power_snapshot()
    print("power_snapshot   :", pow_s)
    ok &= pow_s is not None and pow_s["voltage"] == 7.9

    print("health           :", hub.health())
    ok &= all(hub.health().values())
    hub.stop()

    # A hung poller: snapshot goes stale AND thread_alive() reports it.
    hub2 = SensorHub(imu=HangingIMU())
    hub2.start()
    time.sleep(1.5)
    stale = hub2.imu_snapshot()
    alive = hub2.thread_alive("imu")
    print(f"hung poller      : snapshot={stale}  thread_alive={alive}")
    ok &= stale is None and alive is False
    hub2.stop()

    print("\n" + ("SENSOR HUB OK" if ok else "SELF-TEST FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
