import time

from smbus2 import SMBus

# The power chip on APEX is a TI INA226 (manufacturer ID 0x5449, die ID 0x2260),
# on a module whose shunt is marked R002 (2 milliohm). It replaces the INA219
# this file used to drive: the two chips share the register numbers but NOT the
# scaling, so the old code read the wrong voltage.
#
# Units: get_voltage() volts, get_current() AMPS, get_power() WATTS.
#
# Voltage is the VBUS pin; current is the voltage across IN+/IN- divided by
# SHUNT_OHMS, computed here so the chip's calibration register is not needed.
# Confirm SHUNT_OHMS with a bench test (shunt mV / known amps) before trusting
# the current reading.

SHUNT_OHMS = 0.002

_REG_CONFIG = 0x00
_REG_SHUNT = 0x01       # signed, 2.5 uV per count
_REG_BUS = 0x02         # 1.25 mV per count
_REG_MANUFACTURER = 0xFE
_REG_DIE = 0xFF
_MANUFACTURER_TI = 0x5449
_DIE_INA226 = 0x2260

# 0x4527: fixed bit 14 | 16 averages | 1.1 ms bus | 1.1 ms shunt | continuous
# shunt + bus. (Power-on default is 0x4127, which is 1 average.)
_CONFIG = 0x4527


class INA226:
    def __init__(self, bus_id=1, addr=0x40, shunt_ohms=SHUNT_OHMS):
        """Raises if the chip does not answer or is not an INA226, so pi5_main's
        _bring_up() reports Power FAIL instead of reading garbage."""
        self.bus = SMBus(bus_id)
        self.addr = addr
        self.shunt_ohms = shunt_ohms
        self.available = False
        try:
            if (self._read_reg(_REG_MANUFACTURER) != _MANUFACTURER_TI
                    or self._read_reg(_REG_DIE) != _DIE_INA226):
                raise OSError("chip at 0x%02X is not an INA226" % addr)
            self._write_reg(_REG_CONFIG, _CONFIG)
        except OSError as e:
            self.bus.close()
            raise OSError(f"INA226 not found on bus {bus_id} @ 0x{addr:02X}: {e}") from e
        self.available = True

    def _write_reg(self, reg, val):
        # 16-bit registers are big-endian on the wire.
        self.bus.write_i2c_block_data(self.addr, reg, [(val >> 8) & 0xFF, val & 0xFF])

    def _read_reg(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return (data[0] << 8) | data[1]

    def get_voltage(self):
        """Bus voltage at the VBUS pin, volts."""
        if not self.available:
            return 5.0
        return self._read_reg(_REG_BUS) * 0.00125

    def get_shunt_mv(self):
        """Voltage across IN+/IN-, millivolts (signed)."""
        raw = self._read_reg(_REG_SHUNT)
        if raw > 32767:
            raw -= 65536
        return raw * 0.0025

    def get_current(self):
        """Current through the shunt, amps (signed; positive IN+ -> IN-)."""
        if not self.available:
            return 0.0
        return (self.get_shunt_mv() / 1000.0) / self.shunt_ohms

    def get_power(self):
        """Voltage at VBUS times shunt current, watts."""
        if not self.available:
            return 0.0
        return self.get_voltage() * self.get_current()


# pi5_main still imports the old name.
INA219 = INA226


if __name__ == "__main__":
    # Read-only live view:  python3 power_monitor.py
    dev = INA226(bus_id=3)
    print("INA226 ok, shunt %.4f ohm" % dev.shunt_ohms)
    while True:
        print("bus %.3f V   shunt %+.3f mV   current %+.3f A   power %+.2f W"
              % (dev.get_voltage(), dev.get_shunt_mv(), dev.get_current(), dev.get_power()))
        time.sleep(0.5)
