import math
import microcontroller
from collections import deque
from adafruit_bno08x import BNO08X
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_blinka.microcontroller.generic_linux.i2c as os_i2c

class AdafruitI2CContainerWrapper:
    """Wraps native Linux I2C to expose the API interface expected by Adafruit libraries."""
    def __init__(self, bus_id):
        self._linux_i2c = os_i2c.I2C(bus_id)

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def readfrom_into(self, address, buffer, **kwargs):
        return self._linux_i2c.readfrom_into(address, buffer, **kwargs)

    def writeto(self, address, buffer, **kwargs):
        return self._linux_i2c.writeto(address, buffer, **kwargs)

    def writeto_then_readfrom(self, address, buffer_out, buffer_in, **kwargs):
        return self._linux_i2c.writeto_then_readfrom(address, buffer_out, buffer_in, **kwargs)

class IMU:
    def __init__(self, sda_pin, scl_pin, bus_id=13, window_size=10):
        """
        CPython IMU Handler for Pi 5 container environments.
        """
        # Kept exactly as requested for main file configuration tracking
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        
        # Wrap the functional hardware bus to satisfy Adafruit's library constraints
        self.i2c = AdafruitI2CContainerWrapper(bus_id)
        self.bno = BNO08X_I2C(self.i2c)
        
        # Access constants safely from the BNO08X class
        self.bno.enable_feature(BNO08X.REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(BNO08X.REPORT_ROTATION_VECTOR)

        self.history_roll = deque(maxlen=window_size)
        self.history_pitch = deque(maxlen=window_size)
        self.history_accel_x = deque(maxlen=window_size)
        self.history_accel_y = deque(maxlen=window_size)
        self.history_accel_z = deque(maxlen=window_size)

    def _quat_to_pitch_roll(self, i, j, k, real):
        sinr_cosp = 2 * (real * i + j * k)
        cosr_cosp = 1 - 2 * (i * i + j * j)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (real * j - k * i)
        pitch = math.degrees(math.asin(sinp)) if abs(sinp) < 1 else math.degrees(math.copysign(math.pi / 2, sinp))
        return roll, pitch

    def update(self):
        try:
            quat = self.bno.quaternion 
            r, p = self._quat_to_pitch_roll(*quat)
            ax, ay, az = self.bno.linear_acceleration

            self.history_roll.append(r)
            self.history_pitch.append(p)
            self.history_accel_x.append(ax)
            self.history_accel_y.append(ay)
            self.history_accel_z.append(az)

            return {
                "roll": sum(self.history_roll) / len(self.history_roll),
                "pitch": sum(self.history_pitch) / len(self.history_pitch),
                "accel": (sum(self.history_accel_x)/len(self.history_accel_x), 
                          sum(self.history_accel_y)/len(self.history_accel_y), 
                          sum(self.history_accel_z)/len(self.history_accel_z))
            }
        except Exception as e:
            print(f"BNO085 Error: {e}")
            return None

if __name__ == "__main__":
    print("IMU.py standalone import test passed successfully.")