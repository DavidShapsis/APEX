"""
boot_display.py -- optional SH1106 128x64 OLED for boot progress and status.

Purely additive. If ``luma.oled`` is not installed, or the panel is not on the
bus, every method here becomes a no-op that still echoes to stdout -- so this
module can never be the thing that stops the robot booting, which would defeat
its entire purpose.

Hardware: a 1.3" I2C OLED with the **SH1106** controller (NOT SSD1306 -- the
SSD1306 driver leaves a 2-pixel column offset on these panels). 4-pin module,
address 0x3C, wired to the Pi's primary I2C bus (``/dev/i2c-1``, GPIO2/GPIO3),
sharing it with the compass at 0x0D -- no address clash. Full pinout and the
one-line install are in ``HARDWARE.md`` at the repo root.

    pip install luma.oled        # pulls luma.core + pillow

Usage from pi5_main:

    disp = BootDisplay()                 # safe even with nothing plugged in
    disp.step("IMU")                     # "IMU ..."
    disp.ok("IMU")                       # "IMU            OK"
    disp.fail("Camera")                  # "Camera         FAIL"
    disp.summary(ip, {"IMU": True, ...}) # full status screen, call periodically
"""

import socket
import subprocess
import threading
import time

# 128x64, ~6x8 default font -> 21 chars per row. Under the heading (rows start
# at y=13, 11 px pitch, last row must start by y=56) exactly 4 rows fit:
# 13, 24, 35, 46. Keeping 5 log lines cut off the NEWEST one.
_WIDTH, _HEIGHT = 128, 64
_LINE_PX = 11
_MAX_LOG = 4
_CHARS = 21


def _my_ip():
    """Best-effort primary IP.

    The UDP 'connect' sends nothing -- it just asks the kernel which source
    address a packet to that host would use. It needs a route, though, and a Pi
    running as its own hotspot usually has none, so fall back to the addresses
    actually configured on the interfaces.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        pass
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True,
                             timeout=1.0).stdout
        for addr in out.split():
            if "." in addr and not addr.startswith("127."):
                return addr
    except (OSError, subprocess.SubprocessError):
        pass
    return "no network"


class BootDisplay:
    def __init__(self, port=1, address=0x3C, echo=True):
        self.echo = echo
        self._dev = None
        self._font = None
        self._canvas = None
        self._log = []
        self._lock = threading.Lock()
        # Serialises the bus: boot lines paint on the caller's thread, the
        # running status on the worker below.
        self._dev_lock = threading.Lock()
        self._pending = None
        self._wake = threading.Event()
        self._worker = None
        self._closed = False

        try:
            from luma.core.interface.serial import i2c
            from luma.core.render import canvas
            from luma.oled.device import sh1106
            from PIL import ImageFont

            self._dev = sh1106(i2c(port=port, address=address))
            # luma blanks the panel in an atexit hook unless persist is set.
            # Keep the last frame instead: a boot crash should leave its FATAL
            # line visible, and a normal exit paints "stopped" via shutdown().
            self._dev.persist = True
            self._canvas = canvas
            try:
                self._font = ImageFont.load_default()
            except Exception:
                self._font = None
            self._echo("[OLED] SH1106 ready on i2c-%d @ 0x%02X" % (port, address))
        except Exception as e:
            self._dev = None
            self._echo("[OLED] not available (%s) -- status to stdout only" % e)

    # -- low level --------------------------------------------------------

    def _echo(self, text):
        if self.echo:
            print(text)

    def _paint(self, rows, heading=None):
        """rows: list[str]. Rendered top-down. Never raises."""
        if self._dev is None:
            return
        try:
            with self._dev_lock, self._canvas(self._dev) as draw:
                y = 0
                if heading is not None:
                    draw.text((0, 0), heading[:_CHARS], font=self._font, fill="white")
                    draw.line((0, 10, _WIDTH - 1, 10), fill="white")
                    y = 13
                for row in rows:
                    if y > _HEIGHT - 8:
                        break
                    draw.text((0, y), str(row)[:_CHARS], font=self._font, fill="white")
                    y += _LINE_PX
        except Exception:
            # A bus glitch on a shared line must not take down the caller.
            pass

    # -- boot log --------------------------------------------------------

    def line(self, text):
        """Append a line to the rolling boot log and repaint."""
        self._echo("[BOOT] %s" % text)
        with self._lock:
            self._log.append(text)
            self._log = self._log[-_MAX_LOG:]
            rows = list(self._log)
        self._paint(rows, heading="APEX  booting")

    def step(self, name):
        self.line("%s ..." % name)

    def ok(self, name):
        self.line("%-14s OK" % name[:14])

    def fail(self, name):
        self.line("%-14s FAIL" % name[:14])

    # -- running status -------------------------------------------------

    def summary(self, ip=None, health=None, extra=None):
        """Full status screen. Call every couple of seconds from the main loop.

        health: {"IMU": True, "GPS": False, ...}. extra: optional single line
        (e.g. "NAV: waypoint 2/3").

        Returns immediately: the IP lookup and the I2C repaint (a full frame is
        on the order of 100 ms at the Pi's default 100 kHz bus) run on a
        background thread, so the control loop never waits on the panel. If
        calls arrive faster than the panel can paint, only the latest is drawn.
        """
        if self._dev is None or self._closed:
            return
        with self._lock:
            self._pending = (ip, dict(health) if health else None, extra)
            if self._worker is None:
                self._worker = threading.Thread(target=self._run, name="oled",
                                                daemon=True)
                self._worker.start()
        self._wake.set()

    def _run(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                job, self._pending = self._pending, None
            if job is not None and not self._closed:
                self._paint_summary(*job)

    def _paint_summary(self, ip, health, extra):
        if ip is None:
            ip = _my_ip()
        # No "http://": with it, a typical address overflows the 21-char row
        # and the port gets cut off. ip:port is at most 20 chars.
        rows = ["%s:5000" % ip]
        if health:
            downs = [k for k, v in health.items() if not v]
            if downs:
                # Wrap the down-list across lines so all of it shows.
                label = "DOWN: " + ", ".join(downs)
                while label:
                    rows.append(label[:_CHARS])
                    label = label[_CHARS:]
            else:
                rows.append("all systems nominal")
        if extra:
            rows.append(str(extra))
        self._paint(rows, heading="APEX")

    def clear(self):
        self._paint([])

    def shutdown(self, reason="stopped"):
        """Final frame, painted synchronously and left on the panel. After this
        no status update can overwrite it, so a frozen screen never looks live."""
        with self._lock:
            self._closed = True
            self._pending = None
        self._paint([reason, time.strftime("at %H:%M")], heading="APEX")


if __name__ == "__main__":
    d = BootDisplay()
    for name in ("IMU", "GPS", "Compass", "Power", "Camera", "Dashboard"):
        d.step(name)
        time.sleep(0.3)
        (d.ok if name != "Camera" else d.fail)(name)
        time.sleep(0.3)
    time.sleep(0.5)
    d.summary(health={"IMU": True, "GPS": True, "Compass": True,
                      "Power": True, "Camera": False, "Audio": True},
              extra="NAV: idle")
    time.sleep(0.5)   # summary paints on a background thread
    print("boot_display self-test done"
          " (no-op if no panel is attached)")
