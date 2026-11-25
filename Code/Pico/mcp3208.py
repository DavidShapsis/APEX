from machine import Pin, SPI


class MCP3208:
    """Microchip MCP3208: 8-channel, 12-bit SAR ADC over SPI.

    Default pins are the RP2040's hardware SPI0 on GP16-19, which is how every
    leg board wires its MCP3208 (DOUT->GP16, CS->GP17, CLK->GP18, DIN->GP19).
    """

    # 1 MHz is the datasheet ceiling at VDD = 2.7 V, so it is safe across the
    # whole supply range. The floor is ~10 kHz: below that the sample cap
    # droops before the 12 bits are clocked out.
    BAUDRATE = 1_000_000
    FULL_SCALE = 4096

    def __init__(self, spi_id=0, sck=18, mosi=19, miso=16, cs=17, vref=3.3,
                 baudrate=BAUDRATE):
        self._spi = SPI(spi_id, baudrate=baudrate, polarity=0, phase=0,
                        bits=8, firstbit=SPI.MSB,
                        sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
        # CS must start high: the chip only begins a conversion on a falling CS.
        self._cs = Pin(cs, Pin.OUT, value=1)
        self.vref = vref
        # Preallocated so a read never allocates -- it runs every control-loop pass.
        self._tx = bytearray(3)
        self._rx = bytearray(3)

    def read_raw(self, channel):
        """Single-ended conversion on `channel` (0-7). Returns 0-4095."""
        if not 0 <= channel <= 7:
            raise ValueError("MCP3208 channel must be 0-7")
        # Datasheet fig. 6-1 framing: five leading zeros, start bit, SGL=1, then
        # D2 D1 D0. That leaves the null bit and B11-B8 in the low nibble of the
        # second reply byte and B7-B0 in the third.
        self._tx[0] = 0x06 | (channel >> 2)
        self._tx[1] = (channel & 0x03) << 6
        self._tx[2] = 0
        self._cs(0)
        try:
            self._spi.write_readinto(self._tx, self._rx)
        finally:
            self._cs(1)
        return ((self._rx[1] & 0x0F) << 8) | self._rx[2]

    def read_voltage(self, channel):
        return self.read_raw(channel) * self.vref / self.FULL_SCALE
