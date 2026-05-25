from smbus2 import SMBus

class INA219:
    def __init__(self, bus_id=1, addr=0x40):
        try:
            self.bus = SMBus(bus_id)
            self.addr = addr
            # Configuration: 32V range, +/-320mV shunt range, 12-bit ADC
            self._write_reg(0x00, 0x399F) 

            self._write_reg(0x05, 2048)
            self.available = True
        except Exception as e:
            print(f"INA219 initialization failed: {e}")
            self.available = False

    def _write_reg(self, reg, val):
        # Pi 5 Big-Endian handling for 16-bit registers
        bus_data = [(val >> 8) & 0xFF, val & 0xFF]
        self.bus.write_i2c_block_data(self.addr, reg, bus_data)

    def _read_reg(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return (data[0] << 8) | data[1]

    def get_voltage(self):
        if not self.available: return 5.0
        raw = self._read_reg(0x02)
        return (raw >> 3) * 0.004

    def get_current(self):
        """Returns Current in Milliamps (mA)"""
        if not self.available: return 0.0
        raw = self._read_reg(0x04)
        if raw > 32767: raw -= 65536
        return raw * 0.2

    def get_power(self):
        if not self.available: return 0.0
        raw = self._read_reg(0x03)
        return raw * 2.0