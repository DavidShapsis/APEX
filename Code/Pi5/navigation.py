import math
import threading
import time

import serial
from smbus2 import SMBus

class GPSReader:
    def __init__(self, uart_path, baudrate=115200):
        """
        :param uart_path: The Linux device path (e.g. /dev/ttyAMA4)
        """
        self.ser = serial.Serial(uart_path, baudrate, timeout=0.1)
        self.lat, self.lon = 0.0, 0.0
        self.has_fix = False
        self.satellites = 0
        self._rx = b''
        # monotonic time of the last GGA that carried a usable position. A GPS
        # that goes silent (unplugged, lost power) leaves has_fix / lat / lon
        # frozen at their last values -- this is how a caller tells the fix is
        # stale rather than current.
        self.fix_t = 0.0

    def update(self):
        """Drains the GPS UART without blocking. True if a fresh position parsed.

        readline() is unusable here: in_waiting > 0 does not mean a whole NMEA
        sentence has arrived, so a partial one blocks for the full timeout inside
        the 100Hz control loop.
        """
        updated = False
        try:
            pending = self.ser.in_waiting
            if pending:
                self._rx += self.ser.read(pending)
        except Exception:
            return False

        while b'\n' in self._rx:
            raw, self._rx = self._rx.split(b'\n', 1)
            line = raw.decode('ascii', errors='replace')
            if 'GGA' not in line:
                continue
            try:
                parts = line.split(',')
                if len(parts) > 7:
                    # Fix quality (part 6) is read even when lat/lon are blank,
                    # so a lost fix clears the flag instead of leaving it stale.
                    self.has_fix = bool(parts[6]) and int(parts[6]) > 0
                    self.satellites = int(parts[7]) if parts[7] else 0

                    # Latitude is part 2 / direction 3, longitude 4 / direction 5
                    if self.has_fix and parts[2] and parts[4]:
                        self.lat = self.convert_to_decimal(parts[2], parts[3])
                        self.lon = self.convert_to_decimal(parts[4], parts[5])
                        self.fix_t = time.monotonic()
                        updated = True
            except Exception:
                pass
        return updated

    def convert_to_decimal(self, raw_value, direction):
        if not raw_value or not direction:
            return 0.0
        
        # NMEA format is DDMM.MMMMM for Lat and DDDMM.MMMMM for Lon
        # Find the decimal point to separate degrees from minutes
        dot_index = raw_value.find('.')
        if dot_index == -1:
            return 0.0
        
        # The two digits immediately before the decimal are always the start of 'Minutes'
        degrees = float(raw_value[:dot_index-2])
        minutes = float(raw_value[dot_index-2:])
        
        decimal_degrees = degrees + (minutes / 60.0)
        
        # West and South must be negative for Google Maps
        if direction in ['W', 'S']:
            decimal_degrees *= -1
            
        return round(decimal_degrees, 8)

class CompassReader:
    # QMC5883L status register (0x06) bits.
    _STATUS_DRDY = 0x01
    _STATUS_OVL = 0x02      # any axis saturated -> the reading is pinned garbage

    def __init__(self, sda_pin, scl_pin, explicit_bus_id=None, declination_deg=0.0):
        """
        Determines Bus ID with explicit override support for custom Linux I2C buses.

        declination_deg: magnetic declination at the operating area, EAST
        positive. The chip measures magnetic north; GPS waypoint bearings (and
        coordinates copied from Google Maps) are true north. get_heading() adds
        this so everything downstream is in true north. See MAGNETIC_DECLINATION_DEG
        in pi5_main.
        """
        if explicit_bus_id is not None:
            self.bus_id = explicit_bus_id
        else:
            self.bus_id = 1 if sda_pin == 2 else 0

        self.bus = SMBus(self.bus_id)
        self.addr = 0x0D
        self.declination_deg = declination_deg
        self.configured = False
        try:
            self.bus.write_byte_data(self.addr, 0x0A, 0x80)   # soft reset
            time.sleep(0.01)
            self.bus.write_byte_data(self.addr, 0x0B, 0x01)   # SET/RESET period (recommended)
            # 0x1D = OSR 512 | RNG 8G | ODR 200Hz | continuous
            self.bus.write_byte_data(self.addr, 0x09, 0x1D)
            self.configured = True
        except OSError:
            print(f"Compass not found on Bus {self.bus_id}")

    def get_heading(self, roll_deg=0.0, pitch_deg=0.0):
        """True-north heading in degrees, or None if the read failed or the
        sample is unusable. None (not 0.0) so a wedged bus is distinguishable
        from a genuine due-north reading -- SensorHub holds the last good
        heading across a None, and the control loop stops driving on a run of
        them.

        roll_deg / pitch_deg (from the IMU) tilt-compensate the reading. The
        body pitches and rolls every gait step; without this the heading
        oscillates with the stride. When both are 0 the result is identical to
        the old flat-mount atan2(y, x). Frame assumed: x forward, y right,
        z down, roll about x, pitch about y -- verify the signs if a bench
        heading check ever shows the turn direction inverted.
        """
        try:
            data = self.bus.read_i2c_block_data(self.addr, 0x00, 7)
        except OSError:
            return None

        if data[6] & self._STATUS_OVL:
            return None
        x = self._convert(data[0], data[1])
        y = self._convert(data[2], data[3])
        z = self._convert(data[4], data[5])
        if (x, y, z) in ((0, 0, 0), (-1, -1, -1)):
            # all 0x0000 or all 0xFFFF -> chip not really answering
            return None

        # Tilt compensation, Honeywell AN3192 form. De-rotates the field into
        # the horizontal plane before the atan2. At roll = pitch = 0 this
        # reduces exactly to atan2(y, x) -- identical to the old flat-mount code.
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        xh = x * cp + y * sr * sp + z * cr * sp
        yh = y * cr - z * sr
        heading = math.degrees(math.atan2(yh, xh))
        return (heading + self.declination_deg + 360.0) % 360.0

    def _convert(self, lsb, msb):
        val = lsb | (msb << 8)
        return val if val < 32768 else val - 65536

class Navigator:
    """Waypoint follower. The list can be replaced at runtime from the
    dashboard, so every access is under a lock -- the control loop reads it at
    ~100 Hz while a ROS executor thread may be rewriting it."""

    # How close counts as "arrived", metres.
    ARRIVE_RADIUS_M = 4.0

    def __init__(self, waypoints=None):
        self._lock = threading.Lock()
        self.waypoints = list(waypoints or [])
        self.wp_idx = 0

    # -- runtime editing --------------------------------------------------

    def set_waypoints(self, waypoints):
        """Replace the whole route and restart it. Returns the accepted list.

        Rejects anything outside real lat/lon range rather than trusting the
        dashboard: a bad pair here becomes a confident bearing to nowhere.
        """
        clean = []
        for pair in waypoints or []:
            try:
                lat, lon = float(pair[0]), float(pair[1])
            except (TypeError, ValueError, IndexError):
                continue
            if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
                clean.append((lat, lon))
        with self._lock:
            self.waypoints = clean
            self.wp_idx = 0
        return clean

    def get_waypoints(self):
        with self._lock:
            return list(self.waypoints)

    def reset_progress(self):
        """Rewind to the first waypoint without changing the list. Used by the
        dashboard's Start and Stop so a route can be replayed from the top."""
        with self._lock:
            self.wp_idx = 0

    def progress(self):
        """(index of the waypoint being driven to, total). index == total means
        the route is finished."""
        with self._lock:
            return min(self.wp_idx, len(self.waypoints)), len(self.waypoints)

    @property
    def mission_complete(self):
        """True when there is nothing left to drive to -- either the route ran
        out or none was ever set. The caller must stop; see pi5_main."""
        with self._lock:
            return self.wp_idx >= len(self.waypoints)

    # -- guidance ---------------------------------------------------------

    def calculate_nav(self, curr_lat, curr_lon, curr_head):
        """Bearing error and distance to the active waypoint, or None once the
        route is finished. None means STOP, not "carry on as you were" -- see
        the mission-end handling in pi5_main's control loop."""
        with self._lock:
            if self.wp_idx >= len(self.waypoints):
                return None
            target_lat, target_lon = self.waypoints[self.wp_idx]

        rad_lat1, rad_lat2 = math.radians(curr_lat), math.radians(target_lat)
        d_lon = math.radians(target_lon - curr_lon)
        y = math.sin(d_lon) * math.cos(rad_lat2)
        x = math.cos(rad_lat1) * math.sin(rad_lat2) - math.sin(rad_lat1) * math.cos(rad_lat2) * math.cos(d_lon)
        target_bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        turn_error = target_bearing - curr_head
        if turn_error > 180:
            turn_error -= 360
        if turn_error < -180:
            turn_error += 360

        acos_arg = (math.sin(rad_lat1)*math.sin(rad_lat2) + math.cos(rad_lat1)*math.cos(rad_lat2) * math.cos(d_lon))
        dist = math.acos(max(-1.0, min(1.0, acos_arg))) * 6371000
        if dist < self.ARRIVE_RADIUS_M:
            with self._lock:
                self.wp_idx += 1
        return {"turn": turn_error, "dist": dist}
