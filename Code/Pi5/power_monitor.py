from smbus2 import SMBus

# NOTE: as wired on APEX this is used as a bus-voltage monitor only. Whether a
# shunt resistor is actually in the load path across VIN+/VIN- is unconfirmed,
# so get_current() / get_power() are kept but nothing calls them (sensor_hub
# reads voltage only). Confirm the wiring, then re-enable the current line in
# sensor_hub._power_poll and the current check in pi5_main. See KNOWN_ISSUES.


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
        """Current in mA. NOT CALLED in the current build -- only meaningful if a
        shunt is in the load path across VIN+/VIN-, which is unconfirmed. The
        config register sets +/-320mV over an assumed 0.1 ohm shunt = +/-3.2A FSR."""
        if not self.available: return 0.0
        raw = self._read_reg(0x04)
        if raw > 32767: raw -= 65536
        return raw * 0.2

    def get_power(self):
        """Power in mW. NOT CALLED -- depends on the shunt/current path being
        real; see get_current()."""
        if not self.available: return 0.0
        raw = self._read_reg(0x03)
        # Power LSB is 20x the current LSB: 20 * 0.2mA * 1V = 4.0 mW
        return raw * 4.0