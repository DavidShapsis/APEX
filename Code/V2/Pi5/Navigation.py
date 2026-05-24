import math
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

    def update(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('ascii', errors='replace')
                if 'GGA' in line:
                    parts = line.split(',')
                    if len(parts) > 5 and parts[2] and parts[4]:
                        # Latitude is part 2, Direction is part 3
                        self.lat = self.convert_to_decimal(parts[2], parts[3])
                        
                        # Longitude is part 4, Direction is part 5
                        self.lon = self.convert_to_decimal(parts[4], parts[5])
                        
                        # Quality/Fix status (1 or 2 means we have a fix)
                        self.has_fix = int(parts[6]) > 0
                        
                        # Satellites is part 7
                        self.satellites = int(parts[7])
                        
                        return True
            except: pass
        return False

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
    def __init__(self, sda_pin, scl_pin, explicit_bus_id=None):
        """
        Determines Bus ID with explicit override support for custom Linux I2C buses.
        """
        if explicit_bus_id is not None:
            self.bus_id = explicit_bus_id
        else:
            self.bus_id = 1 if sda_pin == 2 else 0
            
        self.bus = SMBus(self.bus_id)
        self.addr = 0x0D
        try:
            self.bus.write_byte_data(self.addr, 0x09, 0x1D)
            self.bus.write_byte_data(self.addr, 0x0B, 0x01)
        except: print(f"Compass not found on Bus {self.bus_id}")

    def get_heading(self):
        try:
            data = self.bus.read_i2c_block_data(self.addr, 0x00, 6)
            x = self._convert(data[0], data[1])
            y = self._convert(data[2], data[3])
            return (math.degrees(math.atan2(y, x)) + 360) % 360
        except: return 0.0

    def _convert(self, lsb, msb):
        val = lsb | (msb << 8)
        return val if val < 32768 else val - 65536

class Navigator:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.wp_idx = 0

    def calculate_nav(self, curr_lat, curr_lon, curr_head):
        if self.wp_idx >= len(self.waypoints): return None
        target_lat, target_lon = self.waypoints[self.wp_idx]
        
        rad_lat1, rad_lat2 = math.radians(curr_lat), math.radians(target_lat)
        d_lon = math.radians(target_lon - curr_lon)
        y = math.sin(d_lon) * math.cos(rad_lat2)
        x = math.cos(rad_lat1) * math.sin(rad_lat2) - math.sin(rad_lat1) * math.cos(rad_lat2) * math.cos(d_lon)
        target_bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        turn_error = target_bearing - curr_head
        if turn_error > 180: turn_error -= 360
        if turn_error < -180: turn_error += 360

        dist = math.acos(math.sin(rad_lat1)*math.sin(rad_lat2) + 
                         math.cos(rad_lat1)*math.cos(rad_lat2) * math.cos(d_lon)) * 6371000
        if dist < 1.5: self.wp_idx += 1 
        return {"turn": turn_error, "dist": dist}