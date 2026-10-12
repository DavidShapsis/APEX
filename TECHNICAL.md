# A.P.E.X. — Technical Overview

How APEX works under the hood. For what it is and how to run it, see the [README](README.md). Open issues and bench checklists are in [ENGINEERING_NOTES.md](ENGINEERING_NOTES.md); pin wiring is in [HARDWARE.md](HARDWARE.md).

## Contents

- [Software Architecture](#software-architecture)
- [Pi to Pico Protocol](#pi-to-pico-protocol)
- [Kinematics](#kinematics)
- [Gait](#gait)
- [Control Loop](#control-loop)
- [Obstacle Avoidance](#obstacle-avoidance)
- [Dashboard](#dashboard)
- [Motor Stall Protection](#motor-stall-protection)
- [Current Sensing and Foot Contact](#current-sensing-and-foot-contact)

---

## Software Architecture
 
```
Code/
├── Pi5/                       Raspberry Pi 5 (ROS 2)
│   ├── pi5_main.py            main control loop and robot state
│   ├── sensor_hub.py          threaded sensor polling
│   ├── imu.py                 BNO085 roll/pitch
│   ├── navigation.py          GPS, compass, waypoints
│   ├── power_monitor.py       INA219 battery voltage
│   ├── webcam.py              USB camera
│   ├── vision_obstacle.py     depth-model obstacle avoidance
│   ├── stream_server.py       Flask dashboard server
│   ├── dashboard_page.py      dashboard HTML/CSS/JS
│   ├── dashboard_preview.py   dashboard mock, no hardware
│   ├── boot_display.py        optional status OLED
│   ├── audio.py               speaker alerts (+ *.wav sounds)
│   ├── .env.example           template for the gitignored .env (speaker MAC)
│   ├── single_leg_test.py     one-leg test, no ROS needed
│   ├── test_pi_quadruped.py   unit tests
│   ├── stream_tester.py       camera bench script
│   ├── requirements.txt       pip dependencies
│   ├── ros_development.txt    ROS workflow cheat sheet
│   ├── inverse_kinematics/
│   │   ├── ik_and_gait.py     IK/FK and gait generation
│   │   ├── quadruped_sim.py   PC gait simulator
│   │   └── gait_testing2.py, gait_testing3.py   old gait plots
│   ├── static/
│   │   ├── bootstrap.min.css  vendored Bootstrap
│   │   └── README.md          why it's vendored
│   └── vision_test/
│       └── obstacle_avoidance_test.ipynb   vision tuning notebook
│
└── Pico/                      RP2040 leg controllers (MicroPython, x4)
    ├── pico_main.py           UART receiver + PID loop
    ├── motor_control.py       BTS7960 PID joint control
    ├── mcp3208.py             MCP3208 SPI ADC driver
    ├── current_sense.py       per-joint motor current from the BTS7960 IS pins
    ├── contact_detect.py      early-touchdown detection from motor current
    ├── BTS7960_Test.py        bench test: one H-bridge
    └── single_motor_test.py   bench test: one motor
```
 
## Pi to Pico Protocol
 
The Pi sends gait data over UART to each Pico using a binary protocol:
 
- **Start:** `0xAA 0xAA`
- **Home leg:** `0xAB 0xAB` -- zero this leg's encoders to the current physical pose, no payload
- **Stop leg:** `0xAC 0xAC` -- cut motor holding torque, keep encoder tracking, no payload
- **Payload:** one frame of 16-byte steps (`struct.pack('ffff', roll, pitch, knee, is_swing)`): a 20-step gait cycle, or a 41-entry Stand/Go ramp (`RAMP_STEPS + 1`). The Pico rejects frames longer than 48 steps
- **End (cycle):** `0xFF × 16` -- Pico loops the buffer indefinitely (the walking gait)
- **End (one-shot):** `0xFE × 16` -- Pico holds the final step (startup/recovery ramps)
 
Each Pico steps through the gait buffer at 40ms per step. It's a **crawl gait, one leg airborne at a time** (25% swing / 75% stance duty factor) rather than a trot -- the four legs are phase-offset by `[0, N/2, 3N/4, N/4]`, giving lift order FL → RR → FR → RL, with three feet always planted. Each Pico also announces `LEG,<id>` over UART until it's identified, and reports asynchronously:

- `HOMED,<id>` -- homing done
- `ABORTED,<angles>` -- leg stopped, Pi runs recovery; preceded by `STALL,<id>,<joint>` (jammed joint) or `CONTACT,<id>,<joint>` (early touchdown) saying why
- `ADCFAULT,<id>` -- the current-sense ADC reads as dead; that leg has no touchdown check
- `CUR,<id>,<roll R>,<roll L>,<pitch R>,<pitch L>,<knee R>,<knee L>` -- motor currents in mA every 200 ms, `-1` for a dead channel
 
---
 
## Kinematics
 
The IK engine uses a 3-link chain (hip abductor, thigh, shin) solving for roll, pitch, and knee angles given a target foot position in (X, Y, Z):
 
1. **Roll** — solved in the X-Z plane using `atan2` + `acos` geometry on the abductor link
2. **Pitch/Knee** — solved in the virtual leg plane using law of cosines
Forward kinematics is used for the recovery path, reconstructing foot position from joint angles to interpolate back to home stance.
 
Segment lengths (cm): `a = 9.65` (abductor), `b = 26.84` (thigh), `c = 24.37` (shin)
 
---
 
## Gait
 
The gait path is a 20-step cycle, one leg airborne at a time (25% of the cycle) so three feet are always planted:
 
- **Swing phase** (first 25% of the cycle): a true half-ellipse -- the foot sweeps forward and up together as matched cos/sin of the same angle, easing into liftoff and touchdown instead of slamming into them
- **Stance phase** (remaining 75%): the foot travels backward at constant velocity (deliberately linear, not elliptical, so all three planted feet move at the same rate and don't scrub against the ground)
- **Body shift**: the body leans toward the diagonally-opposite hip just before each lift, to keep the centre of mass off the edge of the support triangle (ramped from 2 cm to 4 cm as a turn tightens)
- **IMU levelling**: differential per-leg foot-height correction from roll/pitch, not a common offset
 
### Steering
 
A **body twist**, not stride differencing. `build_gait()` takes a forward stride and a yaw-per-cycle; each planted foot arcs about the body centre (`body_twist_xy_path`), its neutral hip position rotated `± yaw/2` across the stroke, so the four feet phased together rotate the body -- the hip-roll joint carrying the lateral part of each arc. `yaw == 0` is the straight gait, unchanged term for term.
 
`pi5_main` maps the turn command (+ = right; a dashboard control or the GPS heading error) to a yaw rate plus a forward stride that tapers to zero by 90°, so a ±90° command spins in place while a smaller heading error arcs and still advances. On the dashboard the **LEFT / RIGHT** buttons send ±45° (a moderate arc at about half stride) and the slider spans the full −90°…+90°, its ends being a spin in place; the slider tracks the buttons so it always shows the active command. `quadruped_sim.py --report` checks the straight gait *and* a turn sweep: a full spin is ~13°/cycle (~16°/s, a 90° turn in ~5.5 s) with the stability margin, joint rate, and one-leg-airborne crawl all holding.
 
Before any of this runs, the robot must be homed, stood up, and started from the web dashboard -- see `ENGINEERING_NOTES.md` for that flow.
 
---
 
## Control Loop
 
`pi5_main.py` runs a ~100 Hz loop that fuses heading, chooses a steering command (manual, GPS, or the avoidance override), and rebuilds the gait only when the command or attitude actually changes. Everything that could block it is pushed onto its own thread:
 
| Thread | Job |
|---|---|
| Control loop | Steering decision, gait rebuild, Pico ack/recovery handling, status publish |
| Gait serial worker | The single writer to the four Pico UARTs -- gait frames, home/stop commands, recovery |
| `SensorHub` pollers | One each for IMU (50 Hz, I2C0), GPS (USB serial) + compass (I2C3) at 10 Hz, INA219 (I2C3, 1 Hz) |
| Vision worker | Depth inference + costmap, 1-3 fps; not started if the ONNX model isn't installed |
| Flask / ROS executor | Dashboard requests and ROS 2 callbacks |
 
The sensor split matters: those reads are synchronous I2C/UART transactions, and a NAKing or wedged bus blocks the kernel for its timeout. Inline, that used to stall steering and the IMU reflex with it. Now each poller owns its device and publishes a timestamped snapshot; the loop reads the snapshot with a staleness cutoff, so a dead sensor degrades to a safe default (flat attitude, hold last heading, skip the battery check) and a watchdog logs which poller is stuck. `python3 sensor_hub.py` runs a self-test with fake sensors, including a hung-bus case.

The compass reads *true* north — `MAGNETIC_DECLINATION_DEG` in `pi5_main.py` (currently `-13` for northern NJ; change it for your area) is added to every heading, and the reading is tilt-compensated with the live IMU attitude so it doesn't wobble with each stride. Bad samples (I2C error, magnetic overflow, dead chip) return `None`; a compass that stops reading for more than 3 s makes the robot hold position under GPS nav rather than drive on a frozen bearing, and lights the degraded banner. It is **not** hard/soft-iron calibrated — absolute accuracy near the motors is poor, but the nav loop re-derives the bearing from GPS every pass so it still converges.

The GPS is guarded the same way: `GPSReader` timestamps its last positional fix, and a receiver that goes quiet (fix older than 5 s) counts as *no fix* even though `has_fix` is still latched — autonomous nav then holds position instead of driving toward a frozen waypoint bearing. A camera that fails to open raises at boot (marked down, degrade banner) rather than reading as healthy while returning nothing.

The INA219 is read for **voltage only** right now — whether a shunt is actually in the load path for current sensing is unconfirmed, so `get_current()`/`get_power()` are kept but unused rather than feeding a meaningless number into the low-battery alarm. `LOW_VOLT_THRESHOLD` in `pi5_main.py` also needs a bench check (see `ENGINEERING_NOTES.md`) to confirm which rail it's actually measuring.

Every hardware handle is brought up through `_bring_up()`, which reports FAIL (boot log, OLED `DOWN:` line, dashboard degraded banner) whenever the constructor raises — so the IMU, compass, INA219, camera and speaker classes all raise on a device that does not answer rather than returning a half-working object. Repeated main-loop warnings go through `_log_every()`, at most once per 2 s each.

Audio runs inside the ROS container but plays through the host: `audio.py` sends it to the host's PipeWire over local TCP (`PULSE_SERVER=tcp:127.0.0.1:4713`; the container shares the host network but mounts no sockets), and the host keeps the Bluetooth speaker paired. `QuadrupedAudio` checks that the speaker is the default output and fails otherwise. Its MAC comes from `APEX_SPEAKER_MAC` in the gitignored `Code/Pi5/.env`. Setup is in [HARDWARE.md](HARDWARE.md).
 
---
 
## Obstacle Avoidance
 
Camera-only, using a **pretrained** monocular depth model (Depth-Anything-V2-Small, ONNX) — nothing is trained here. Each frame becomes a relative depth map; a band in front of the feet is sliced into 9 angular bins, and each bin is scored by how much of it reads **nearer than the ground does at that image row**. The planner then takes whatever direction the robot wants to travel and returns the nearest one that is actually passable.
 
That row-relative comparison is the important detail. On flat ground the distance to the ground at image row `r` goes as `1/(r - horizon)`, so depth ramps smoothly from far at the top of the frame to near at the bottom — a single absolute threshold flags the ground itself as an obstacle. An upright object instead occupies rows that would otherwise show ground far away, so it stands out sharply against its own row. A uniform slope shifts a whole row together and is correctly ignored.
 
It runs on its own worker thread at 1–3 fps on the Pi 5 CPU, which is ample — the crawl gait only moves at ~16.7 cm/s. No AI accelerator needed.
 
The planner is a committed state machine, not a per-frame reaction:
 
| State | Behaviour |
|---|---|
| `CLEAR` | Corridor open. Navigation steers, full stride |
| `AVOIDING` | Obstacle ahead. Committed to one side, steering to the passable heading nearest the goal, 0.6x stride |
| `CLEARING` | Corridor reopened. Keeps turning while the obstacle is still in frame, then drives straight until the body is past |
| `BLOCKED` | No gap anywhere. Stride drops to zero — the robot **marches in place**, still standing (unlike STOP, which cuts torque) |
| `ESCAPE` | Still blocked. Slow committed arc toward the least-obstructed side |
 
**Why commitment matters:** the naive version oscillates. Steer away from an obstacle, it leaves the field of view, GPS points straight back at it, repeat forever. Latching a side until the detour completes is what prevents that.
 
**Getting back on track is automatic** — `Navigator.calculate_nav` recomputes the bearing from the live GPS fix every pass, so there is no route line to rejoin. Within a detour the planner always picks the passable heading *closest to the goal bearing*, so it drifts back toward the waypoint as soon as the geometry allows.
 
Verified in simulation against the real `Navigator`: obstacles struck by 0.95 m with avoidance off were cleared by 0.25–0.50 m with it on, with zero steering oscillations, at a 3–19% cost in route time.
 
```bash
cd Code/Pi5
python3 vision_obstacle.py --download    # fetch the ~100MB depth model, once
python3 vision_obstacle.py               # planner self-test, no camera needed
python3 vision_obstacle.py --live 0      # live decisions from the camera
```
 
Then toggle **AVOIDANCE** on the dashboard. While it is on, the video feed is overlaid with the detection bins (red = blocked), so the thresholds can be tuned by eye. `Code/Pi5/vision_test/obstacle_avoidance_test.ipynb` is a standalone notebook for the same tuning against still images.
 
**Not yet verified on hardware** — see `ENGINEERING_NOTES.md` for what needs measuring first.
 
---
 
## Dashboard
 
Open `http://<pi-ip>:5000`. Top to bottom: live camera feed, **Steering** (direction readout, LEFT / FWD / RIGHT, the −90…+90 slider), **Startup** (Home ×4 → STAND → GO → STOP), **Navigation** (collapsible GPS routing + obstacle avoidance), then **Debug** per-leg deactivation, and a collapsed **Motor Currents** panel at the bottom: all 12 motors as right-side / left-side driver current (`R A/L A`), `--` for a leg not reporting. Each card's wordy explanation is tucked behind a small **i** button. STAND and GO stay disabled, with a warning banner, until all four legs are homed; status pills up top show homed count / standing / walking.
 
The **Navigation** card holds two collapsible sections, both closed on load. *GPS Waypoints & Routing* opens with a live readout — current position, true-north heading with its compass point, and satellite count, showing *no fix* / *no compass* when either is missing — then the NAV MODE master toggle, a waypoint editor (one latitude box and one longitude box per point — paste a Google Maps "lat, lng" coordinate string into either box and it splits across both — ▲/▼ to reorder, × to remove, **+ Add point**, **Send route** — nothing reaches the robot until pressed), and a **Start / Pause / Stop** transport row greyed out by state: Start runs the route from the first point, Pause holds position without advancing, Stop returns to manual and rewinds. NAV MODE and Start are refused until at least one waypoint has been sent — the toggle greys out and reads *add a waypoint first* — and the controller rejects the switch to autonomous the same way, so a lost or spoofed message can't put the robot in nav mode with nowhere to go. On reaching the last point the robot holds position. *Obstacle Avoidance* has the avoidance toggle and its live state readout.
 
The markup lives in one place — `dashboard_page.py`, a plain string with no imports — so `stream_server.py` (the real node) and `dashboard_preview.py` (a hardware-free mock for working on the UI) render the identical page. Styling is an APEX red/black theme over Bootstrap 5, and Bootstrap is **vendored** at `Code/Pi5/static/bootstrap.min.css` rather than pulled from a CDN, so the page is fully functional when the Pi is its own access point with no route to the internet.
 
---

## Motor Stall Protection

A jammed joint is now caught in software: `JointController` latches a stall when the PID has been saturated for 1.5 s with neither the error falling nor the encoder turning, zeroes that joint's PWM, and raises the existing `ABORTED` path so the Pi runs its recovery. Before this the joint simply held 100% duty until something burned out.

---

## Current Sensing and Foot Contact

Each BTS7960 has two current-sense outputs (`R_IS`, `L_IS`), each a current source at about 1/8500 of the motor current, from whichever side of the driver is conducting. Every leg's Pico reads its six through an **MCP3208** 12-bit ADC on SPI0 (DOUT GP16, CS GP17, CLK GP18, DIN GP19). Each line is loaded by a 680 Ω to ground, which sets the scale and keeps every case, including the chip's 7 mA fault current, inside the ADC's range, then divided by 5.1 kΩ / 10 kΩ. A 9.5 A motor reads about 0.48 V. Wiring is in [HARDWARE.md](HARDWARE.md).

The code is split by job:

- `mcp3208.py` -- `MCP3208`, the SPI driver.
- `current_sense.py` -- `CurrentSense`, one joint: zeroed at boot with the motors off, smoothed (the 1 kHz PWM chops the signal), reported per side and summed, and flagged invalid if it reads full scale (a dead ADC).
- `contact_detect.py` -- `TouchdownDetector`. It assumes a normal gait, upright, so a sustained current spike while a leg should be in the air means its foot touched down early. It ignores the first 40 ms after liftoff and needs the spike to last 10 ms. It does not tell a touchdown from walking into something.

On a detected touchdown the Pico sends `CONTACT` then `ABORTED`, the same recovery path the foot sensor (FSR) used to drive. The FSR itself is gone; its pin is now the ADC's data line. The thresholds (`TOUCHDOWN_AMPS` in `pico_main.py`) are uncalibrated placeholders. See `ENGINEERING_NOTES.md` for calibration and the open items.
