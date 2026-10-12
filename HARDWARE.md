# APEX — pin allocation, the status OLED & current sensing

Where every wire on the Raspberry Pi 5 40‑pin header goes, how to add the
1.3" SH1106 OLED that `Code/Pi5/boot_display.py` drives, and how each leg's
Pico reads its motor currents ([Leg boards](#leg-boards-pico-pins-and-current-sensing)).

> The OLED is **optional**. `boot_display.py` degrades to stdout if the panel or
> the `luma.oled` library is missing, so nothing here is required to run the
> robot — it just means a boot failure is visible without a laptop on the
> serial console.

---

## The screen

**Hosyond / generic 1.3" I2C OLED, 128×64, SH1106 controller.** 4‑pin module,
markings usually `GND VCC SCL SDA`.

- Controller is **SH1106**, *not* SSD1306. The SSD1306 driver runs but leaves a
  ~2‑pixel column offset — use the SH1106 driver (`luma.oled`'s `sh1106` class
  handles the 132‑vs‑128 column RAM offset for you).
- I2C address **0x3C** (fixed on these modules; a few are 0x3D).
- 3–5 V input, 3.3 V logic — power it from **3.3 V** so the SDA/SCL lines stay
  at the Pi's 3.3 V level.

Install the driver on the Pi:

```bash
pip install luma.oled        # pulls in luma.core + pillow
```

---

## What's already used on the header

Each I2C device has a bus of its own except the compass and INA219, which share
I2C3 (no address clash). Derived from the code (`IMU`, `CompassReader`,
`INA219`, `BootDisplay`, `pi5_main.pico_ports`).

| Bus / signal | Device | Addr | Header pins (BCM GPIO) | Code |
|---|---|---|---|---|
| **I2C0** (`/dev/i2c-0`) | BNO085 IMU | `0x4A`/`0x4B` | 27 (GPIO0 SDA), 28 (GPIO1 SCL) | `IMU(bus_id=0)` |
| **I2C1** (`/dev/i2c-1`) | SH1106 OLED | `0x3C` | 3 (GPIO2 SDA), 5 (GPIO3 SCL) | `BootDisplay(port=1)` |
| **I2C3** (`/dev/i2c-3`) | QMC5883L compass (built into the HGLRC M100 GPS) | `0x0D` | 15 (GPIO22 SDA), 16 (GPIO23 SCL) | `CompassReader(explicit_bus_id=3)` |
| **I2C3** (`/dev/i2c-3`) | INA219 power monitor | `0x40` | 15, 16 — in parallel with the compass | `INA219(bus_id=3)` |
| **UART0** (`/dev/ttyAMA0`) | Leg Pico | — | 8 (GPIO14 TX), 10 (GPIO15 RX) | `pico_ports` |
| **UART2** (`/dev/ttyAMA2`) | Leg Pico | — | 7 (GPIO4 TX), 29 (GPIO5 RX) | `pico_ports` |
| **UART3** (`/dev/ttyAMA3`) | Leg Pico | — | 24 (GPIO8 TX), 21 (GPIO9 RX) | `pico_ports` |
| **UART4** (`/dev/ttyAMA4`) | Leg Pico | — | 32 (GPIO12 TX), 33 (GPIO13 RX) | `pico_ports` |
| USB (`/dev/ttyUSB0`) | HGLRC M100 GPS via USB‑TTL adapter, 115200 baud | — | — | `GPSReader` |
| USB (`/dev/video*`) | USB webcam | — | — | `USBWebcam()` auto‑detects |
| Power | — | — | 1 or 17 (3.3 V), 2 or 4 (5 V) | |
| Ground | — | — | 6, 9, 14, 20, 25, 30, 34, 39 | |

Power the IMU and OLED from **3.3 V**. The M100 takes 5 V (from the USB‑TTL
adapter), but its SDA/SCL pull up to its own 3.3 V regulator, so it connects to
the Pi directly with no level shifter — measure SDA/SCL to GND before wiring a
different GPS module, since a 5 V pull‑up would damage the Pi's GPIO.

Each Pico talks on its GP0 (TX) / GP1 (RX); cross them (Pi TX → Pico RX) and
share a ground. Which corner a Pico drives comes from its `LEG_ID` announcement,
not from which UART it is on.

### `/boot/firmware/config.txt` (on the Pi host, not in the container)

```ini
dtparam=i2c_arm=on            # I2C1  — OLED
dtparam=i2c0=on               # I2C0  — IMU, GPIO0/1
dtoverlay=i2c3,pins_22_23     # I2C3  — compass + INA219
dtparam=uart0=on              # ttyAMA0
dtoverlay=uart2               # ttyAMA2
dtoverlay=uart3               # ttyAMA3
dtoverlay=uart4               # ttyAMA4
```

There must be **no** `dtoverlay=uart1`: UART1 claims GPIO0/1 too, and with it
present `/dev/i2c-0` never appears. Nothing uses UART1.

`/dev/i2c-13` and `/dev/i2c-14` also exist on a Pi 5, but they are the HDMI
ports' DDC buses, not header pins — the IMU used to be pointed at `i2c-13`,
which is why it never answered.

Check with `i2cdetect -l` (buses 0, 1, 3 should be listed) and then:

```bash
i2cdetect -y 0     # 4a  (IMU)
i2cdetect -y 1     # 3c  (OLED)
i2cdetect -y 3     # 0d  (compass), 40 (INA219)
```

A device that does not answer is reported as FAIL at boot and as down on the
dashboard and OLED; the robot still boots.

---

## OLED wiring

| OLED pin | → Pi 5 header pin | Signal |
|---|---|---|
| `VCC` | **1** | 3.3 V |
| `GND` | **9** | GND |
| `SCL` | **5** | GPIO3 / SCL1 |
| `SDA` | **3** | GPIO2 / SDA1 |

```
        Pi 5 header (top-left corner)
        ┌───────────────────────
   3V3  │ 1 ●  ● 2   5V
   SDA1 │ 3 ●  ● 4   5V
   SCL1 │ 5 ●  ● 6   GND
        │ 7 ●  ● 8
   GND  │ 9 ●  ● 10
        └───────────────────────
   OLED:  VCC→1   SDA→3   SCL→5   GND→9
```

The default `BootDisplay(port=1, address=0x3C)` is already correct. Check with
`i2cdetect -y 1` (expect `3c`), then `python3 Code/Pi5/boot_display.py`, which
marches a fake boot sequence across the panel. When `pi5_main` exits (Ctrl+C,
`pkill`, `docker stop`) the panel is left showing `stopped` and the time, so a
frozen status frame is never mistaken for a live one.

---

## Leg boards: Pico pins and current sensing

Every leg board is identical: one Pico, three BTS7960 drivers (roll, pitch, knee)
and one MCP3208 ADC. Pins from `Code/Pico/pico_main.py`:

| Pico GPIO | Signal |
|---|---|
| GP0 / GP1 | UART0 TX / RX to the Pi |
| GP3 / GP2, GP6 | roll RPWM / LPWM, EN |
| GP4 / GP5 | roll encoder A / B |
| GP11 / GP10, GP7 | pitch RPWM / LPWM, EN |
| GP8 / GP9 | pitch encoder A / B |
| GP15 / GP14, GP22 | knee RPWM / LPWM, EN |
| GP12 / GP13 | knee encoder A / B |
| GP16 | MCP3208 DOUT (SPI0 RX) |
| GP17 | MCP3208 CS |
| GP18 | MCP3208 CLK (SPI0 SCK) |
| GP19 | MCP3208 DIN (SPI0 TX) |

GP16 used to be the FSR foot sensor; the FSR is gone.

### MCP3208 channels

| Channel | Signal |
|---|---|
| CH0 / CH1 | roll driver `R_IS` / `L_IS` |
| CH2 / CH3 | pitch driver `R_IS` / `L_IS` |
| CH4 / CH5 | knee driver `R_IS` / `L_IS` |
| CH6 / CH7 | not connected |

The joint-to-pair mapping and the R/L order within each pair are
**unconfirmed**. Command one joint to move and see which reading moves (enable
`CURRENT_LOG` or open the dashboard's Motor Currents panel), then fix `channels=`
in `pico_main.py` if needed.

Power the MCP3208 (VDD and VREF) from **3.3 V**. The code assumes VREF = 3.3 V,
and a 5 V supply would also drive GP16 at 5 V, which the Pico can't take.

### Each `IS` line (24 total, 6 per leg)

```
IS pin ──┬── 5.1k ──┬── MCP3208 CHx
         │          │
       680Ω        10k
         │          │
        GND        GND
```

`IS` is a current source, so the **680 Ω is required**: it sets the scale and
keeps every case, including the BTS7960's 7 mA fault current, inside the ADC's
range (full numbers in `ENGINEERING_NOTES.md` → *Ground contact — current
sensing replaced the FSR*). Keep it at or below ~700 Ω.

Optional: a 1 µF capacitor from each MCP3208 input to GND smooths the 1 kHz PWM
ripple (undecided; see `ENGINEERING_NOTES.md`).

### Verify before connecting the ADC

- With power off, each `IS` line to GND should measure **~650 Ω**, or ~610 Ω if
  the driver board has its own 10 kΩ on `IS`. If it does, set `DRIVER_IS_OHMS =
  10000` in `pico_main.py`.
- Powered, with the motors idle, every MCP3208 input should sit near 0 V and
  never above 3.3 V.

---

## Speaker (Bluetooth)

The host owns the Bluetooth link; the container only plays audio. The container
has no host audio socket or D-Bus mounted, but it shares the host network, so
`audio.py` sends sound to the host's PipeWire over local TCP
(`PULSE_SERVER=tcp:127.0.0.1:4713`).

One-time setup on the **host**:

```bash
bluetoothctl trust <speaker MAC>          # bluez reconnects it on its own
mkdir -p ~/.config/pipewire/pipewire-pulse.conf.d
cat > ~/.config/pipewire/pipewire-pulse.conf.d/tcp.conf <<'EOF'
pulse.properties = {
    server.address = [ "unix:native" "tcp:127.0.0.1:4713" ]
}
EOF
systemctl --user restart pipewire-pulse
```

In the **container**: `apt install -y libpulse0 pulseaudio-utils`. Check with
`PULSE_SERVER=tcp:127.0.0.1:4713 pactl get-default-sink` -- it should print
`bluez_output.<MAC with underscores>.1`. Boot reports Audio FAIL if it does not.

The MAC is not in the repo: `pi5_main` reads `APEX_SPEAKER_MAC` from
`Code/Pi5/.env`, which is gitignored. Copy `Code/Pi5/.env.example` to `.env`
next to `pi5_main.py` and fill it in.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `i2cdetect` shows nothing at 0x3C | power/ground swapped, or SDA/SCL swapped; some clones need a moment after power‑on |
| Panel lights but shows garbage / shifted 2 px | SSD1306 driver in use — must be **SH1106** |
| `[OLED] not available (No module named 'luma')` | `pip install luma.oled` on the Pi |
| Works standalone, blank when `pi5_main` runs | address or port wrong in `BootDisplay(...)`, or another process holds the bus |
| `IMU init failed: OSError(121 ...)` | IMU not on I2C0 pins 27/28, or `dtoverlay=uart1` still in config.txt (no `/dev/i2c-0`) |
| A bus shows on the host but not in the container | the container was started before the bus existed; restart it |
