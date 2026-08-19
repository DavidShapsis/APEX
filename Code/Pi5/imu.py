import math
from collections import deque
from adafruit_bno08x import BNO08X
from adafruit_bno08x.i2c import BNO08X_I2C
import busio
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as BlinkaI2C

class IMU:
    def __init__(self, sda_pin, scl_pin, bus_id=13, window_size=10):
        """CPython IMU Handler explicitly routing through hardware bus overrides."""
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        
        # Route through Blinka Linux I2C bus if a specialized overlay id is assigned
        if bus_id is not None:
            self.i2c = BlinkaI2C(bus_id)
            
            # --- FIX: Inject the missing CircuitPython locking API methods ---
            self.i2c.try_lock = lambda: True
            self.i2c.unlock = lambda: None
            # -----------------------------------------------------------------
        else:
            self.i2c = busio.I2C(scl_pin, sda_pin, frequency=400000)
            
        self.bno = BNO08X_I2C(self.i2c)
        
        self.bno.enable_feature(BNO08X.REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(BNO08X.REPORT_ROTATION_VECTOR)

        # Roll/pitch are held as (sin, cos) pairs so the moving average survives
        # the +/-180 wrap; averaging the degree values directly does not.
        self.history_roll = deque(maxlen=window_size)
        self.history_pitch = deque(maxlen=window_size)
        self.history_accel_x = deque(maxlen=window_size)
        self.history_accel_y = deque(maxlen=window_size)
        self.history_accel_z = deque(maxlen=window_size)
        
        self.current_roll = 0.0
        self.current_pitch = 0.0

    @staticmethod
    def _circular_mean(history):
        """Mean of angles stored as (sin, cos) pairs, returned in degrees."""
        sin_sum = sum(v[0] for v in history)
        cos_sum = sum(v[1] for v in history)
        return math.degrees(math.atan2(sin_sum, cos_sum))

    def _quat_to_pitch_roll(self, i, j, k, real):
        """Calculates precise pitch and roll conversions from input quaternion structures."""
        sinr_cosp = 2 * (real * i + j * k)
        cosr_cosp = 1 - 2 * (i * i + j * j)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (real * j - k * i)
        pitch = math.degrees(math.asin(sinp)) if abs(sinp) < 1 else math.degrees(math.copysign(math.pi / 2, sinp))
        return roll, pitch

    def update(self):
        try:
            quat = self.bno.quaternion 
            accel = self.bno.linear_acceleration
            
            if quat is None or accel is None:
                return None 
                
            r, p = self._quat_to_pitch_roll(quat[0], quat[1], quat[2], quat[3])
            ax, ay, az = accel

            r_rad, p_rad = math.radians(r), math.radians(p)
            self.history_roll.append((math.sin(r_rad), math.cos(r_rad)))
            self.history_pitch.append((math.sin(p_rad), math.cos(p_rad)))
            self.history_accel_x.append(ax)
            self.history_accel_y.append(ay)
            self.history_accel_z.append(az)

            self.current_roll = self._circular_mean(self.history_roll)
            self.current_pitch = self._circular_mean(self.history_pitch)

            return {
                "roll": self.current_roll,
                "pitch": self.current_pitch,
                "accel": (sum(self.history_accel_x)/len(self.history_accel_x), 
                          sum(self.history_accel_y)/len(self.history_accel_y), 
                          sum(self.history_accel_z)/len(self.history_accel_z))
            }
        except Exception:
            return None
    
    def get_roll(self):
        return self.current_roll

    def get_pitch(self):
        return self.current_pitch

if __name__ == "__main__":
    print("IMU.py standalone import test passed successfully.")