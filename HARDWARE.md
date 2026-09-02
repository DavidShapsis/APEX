# APEX — Pi 5 pin allocation & the status OLED

Where every wire on the Raspberry Pi 5 40‑pin header goes, and how to add the
1.3" SH1106 OLED that `Code/Pi5/boot_display.py` drives.

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

Derived from the code (`CompassReader`, `INA219`, `IMU`, `pi5_main.pico_ports`).
Buses other than I2C1 and UART0 are enabled by `dtoverlay=` lines you added to
`/boot/firmware/config.txt` — check that file for the exact GPIOs if anything
below doesn't match your board.

| Bus / signal | Device | Addr | Header pins (BCM GPIO) | Notes |
|---|---|---|---|---|
| **I2C1** | QMC5883 compass | `0x0D` | 3 (GPIO2 SDA), 5 (GPIO3 SCL) | The Pi's primary I2C. Always enabled via `dtparam=i2c_arm=on`. |
| **I2C3** (`/dev/i2c-3`) | INA219 power monitor | `0x40` | GPIO4 / GPIO5 by default on Pi 5 (`dtoverlay=i2c3`) — **verify in config.txt** | |
| **I2C‑13** (`/dev/i2c-13`) | BNO085 IMU | `0x4A`/`0x4B` | non‑standard bus you enabled — **check config.txt** | `IMU(bus_id=13)`; the `sda_pin`/`scl_pin` args are unused on this path |
| **UART0** (`/dev/ttyAMA0`) | Leg Pico #0 | — | 8 (GPIO14 TX), 10 (GPIO15 RX) | |
| **UART2/3/4** (`ttyAMA2‑4`) | Leg Picos #1‑3 | — | per your `dtoverlay=uart2/3/4` lines — **check config.txt** | Pi 5 defaults: uart2 = GPIO4/5, uart3 = GPIO8/9, uart4 = GPIO12/13 |
| Power | — | — | 1 or 17 (3.3 V), 2 or 4 (5 V) | |
| Ground | — | — | 6, 9, 14, 20, 25, 30, 34, 39 | |

Nothing else in the code touches GPIO directly (motors and encoders are on the
Picos, not the Pi).

**No I2C device currently answers at 0x3C**, so the OLED can share any of the
I2C buses above without an address clash.

---

## Recommended wiring — hang it on I2C1 with the compass

I2C is a bus: extra devices wire **in parallel** on the same two lines. Putting
the OLED on **I2C1** (pins 3 & 5) needs **zero config changes** — that bus is
already up and proven by the compass — and 0x3C ≠ 0x0D, so they coexist.

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

Pins 3 and 5 already carry the compass. If the compass is on a breakout with
pass‑through pads, tap those. Otherwise use a 2‑into‑1 Dupont splitter or solder
both leads to pins 3/5.

Then in `Code/Pi5/boot_display.py` the default `BootDisplay(port=1, address=0x3C)`
is already correct — nothing to change.

### Verify

```bash
sudo apt install -y i2c-tools      # once
i2cdetect -y 1
```

Expect `0d` (compass) **and** `3c` (OLED):

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- 0d -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
40: 40 -- -- -- ...
```

Then:

```bash
python3 Code/Pi5/boot_display.py     # marches a fake boot sequence across the panel
```

---

## Alternative — a dedicated bus (electrical isolation from the compass)

If you'd rather the OLED not share the compass bus (a wedged OLED could add
`Remote I/O error`s to compass reads — `SensorHub` already tolerates those by
holding the last heading, but still), add a software I2C bus on two free pins.
Append to `/boot/firmware/config.txt`:

```
dtoverlay=i2c-gpio,bus=7,i2c_gpio_sda=16,i2c_gpio_scl=26
```

That puts a new `/dev/i2c-7` on **GPIO16 (pin 36)** and **GPIO26 (pin 37)** —
both unused above, but confirm against your own config.txt first. Reboot, then:

```python
BootDisplay(port=7, address=0x3C)   # in pi5_main.main()
```

Wiring becomes VCC→pin 1, GND→pin 34, SDA→pin 36, SCL→pin 37.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `i2cdetect` shows nothing at 0x3C | power/ground swapped, or SDA/SCL swapped; some clones need a moment after power‑on |
| Panel lights but shows garbage / shifted 2 px | SSD1306 driver in use — must be **SH1106** |
| `[OLED] not available (No module named 'luma')` | `pip install luma.oled` on the Pi |
| Works standalone, blank when `pi5_main` runs | address or port wrong in `BootDisplay(...)`, or another process holds the bus |
| Compass reads get flaky after adding the OLED | move the OLED to a dedicated bus (section above) |
