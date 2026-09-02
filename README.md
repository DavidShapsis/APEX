 **⚠️ This is an ongoing project. The code, CAD, and documentation are all actively being developed and are not final.**
 
---
 
# APEX — Autonomous Precision Exploration
 
A fully custom-built quadruped robot dog designed and coded from scratch.

APEX is designed to walk across varied outdoor terrain using real-time inverse kinematics, stay balanced using an IMU, navigate to GPS waypoints autonomously, and stream live camera footage back to any device over WiFi. The long-term goal is onboard ML-based obstacle avoidance for fully autonomous terrain exploration.
 
![Status](https://img.shields.io/badge/status-in%20progress-yellow)
![Platform](https://img.shields.io/badge/brain-Raspberry%20Pi%205-red)
![Firmware](https://img.shields.io/badge/legs-RP2040%20%C3%974-blue)
![License](https://img.shields.io/badge/license-MIT-green)
 
---
 
## Features
 
- **Inverse Kinematics** — custom 3-link IK engine with forward kinematics for recovery, computing joint angles in real time for all four legs
- **Body-Twist Steering** — each foot arcs about the body centre so the robot yaws in place or blends yaw with forward travel into an arc, recruiting the hip-roll joint into the turn (not tank-style stride differencing)
- **IMU Stabilization** — BNO085 quaternion-based roll/pitch feeds a per-leg differential foot-height correction, so the body actually levels instead of just translating
- **GPS Navigation** — autonomous waypoint following using bearing and distance calculations from a HGLRC M100 GPS module; the route is entered and reordered live from the dashboard, and the robot holds position (marching in place) once the last point is reached
- **Live Video Streaming** — USB webcam feed served over Flask to any device on the same network
- **Mission Control Dashboard** — Flask web UI that walks through homing each leg, standing, and going before the robot is allowed to walk; live camera feed with a steering readout, an obstacle-avoidance toggle, and per-leg debug controls. Served entirely from the Pi (Bootstrap vendored locally, no internet needed in the field)
- **ML Obstacle Avoidance** — pretrained monocular depth model (Depth-Anything-V2-Small, ONNX) turns the webcam feed into a forward costmap and steers around obstacles, layered on top of both manual and GPS waypoint steering. Dashboard toggle
- **Threaded Sensor Polling** — every blocking sensor read (IMU + compass over I2C, GPS UART, INA219) runs on its own poller thread; the control loop only ever reads a lock-protected snapshot, so a stuck bus degrades to a safe default instead of stalling the gait
- **Current Sensing Foot Detection** *(in progress)* — motor current sensing on unexpected ground contact triggers an automatic recovery routine, replacing the FSR-based approach
- **ROS 2 Integration** — inter-node communication via ROS 2 topics for direction commands, navigation mode switching, and the homing/stand/go/stop dashboard controls

---
 
## BOM
 
| Component | Qty | Notes |
|---|---|---|
| Raspberry Pi 5 | 1 | Main brain |
| Raspberry Pi Pico (RP2040) | 4 | One per leg, runs MicroPython |
| GoBilda 5302 Yellow Jacket Motor (99.5:1, 60 RPM) | 12 | 3 per leg |
| BTS7960 43A H-Bridge | 12 | One per joint |
| BNO085 IMU | 1 | Quaternion-based orientation |
| HGLRC M100-5883 GPS/Compass | 1 | Outdoor autonomous nav |
| INA219 Voltage/Current Monitor | 1 | Battery telemetry |
| Carbon Fiber Tube (16x12mm) | — | Lower leg structure |
| Aluminum 6063 Tube (1in OD) | — | Upper leg structure |
| 3S 11.1V LiPo 80C 5Ah | 1 | Motor power |
| 2S 7.6V LiHV 3.5Ah | 1 | Electronics power |
| Custom PCB | 1 | High-current motor control, in progress |
 *BOM subject to change. For the most recent BOM go to [APEX BOM on Google Sheets](https://docs.google.com/spreadsheets/d/1m-T2-i6hj74-5jb7eDYWLMdHvXfgs2ABYw7gbwLl6ZI/edit?usp=sharing)*
---
 
## Software Architecture
 
```
Pi 5 (ROS 2)
├── pi5_main.py          # Main control loop, gait generation, homing/stand/go/stop state
├── inverse_kinematics/
│   ├── ik_and_gait.py   # IK, FK, GaitPath, GaitIK, RecoveryPath, shared leg/body geometry
│   └── quadruped_sim.py # PC-only 4-leg gait simulator (no ROS) -- --report and 3D animation modes
├── sensor_hub.py        # Per-sensor poller threads -- keeps blocking I2C/UART off the control loop
├── boot_display.py      # Optional SH1106 OLED: boot progress + status (no-op if absent)
├── imu.py               # BNO085 quaternion → roll/pitch
├── navigation.py        # GPS parsing, compass, waypoint navigation
├── stream_server.py     # Flask dashboard node -- ROS wiring, routes, /status plumbing
├── dashboard_page.py    # The dashboard HTML/CSS/JS (shared string, zero deps)
├── dashboard_preview.py # Throwaway standalone dashboard mock (Flask only, no hardware)
├── static/              # Vendored assets served at /static/ (bootstrap.min.css)
├── webcam.py            # USB camera capture
├── vision_obstacle.py   # Depth model, obstacle costmap, committed avoidance planner
├── vision_test/         # Standalone notebook for tuning the vision pipeline
├── power_monitor.py     # INA219 voltage/current
├── audio.py             # Bluetooth speaker alerts
└── single_leg_test.py   # Standalone single-leg test harness (no ROS/IMU/GPS)
 
Pico (MicroPython, x4)
├── pico_main.py         # UART receiver, gait buffer, PID execution loop, homing/stop commands
├── motor_control.py     # BTS7960 PID joint controller with encoder feedback
└── fsr.py               # Force sensitive resistor foot contact (being replaced by current sensing)
```
 
### Pi to Pico Protocol
 
The Pi sends gait data over UART to each Pico using a binary protocol:
 
- **Start:** `0xAA 0xAA`
- **Home leg:** `0xAB 0xAB` -- zero this leg's encoders to the current physical pose, no payload
- **Stop leg:** `0xAC 0xAC` -- cut motor holding torque, keep encoder tracking, no payload
- **Payload:** up to 20 steps (gait) or 40 (ramp) x 16 bytes each (`struct.pack('ffff', roll, pitch, knee, is_swing)`)
- **End (cycle):** `0xFF × 16` -- Pico loops the buffer indefinitely (the walking gait)
- **End (one-shot):** `0xFE × 16` -- Pico holds the final step (startup/recovery ramps)
 
Each Pico steps through the gait buffer at 40ms per step. It's a **crawl gait, one leg airborne at a time** (25% swing / 75% stance duty factor) rather than a trot -- the four legs are phase-offset by `[0, N/2, 3N/4, N/4]`, giving lift order FL → RR → FR → RL, with three feet always planted. Each Pico also announces `LEG,<id>` over UART until it's identified, and reports `HOMED,<id>` / `ABORTED,<angles>` asynchronously.
 
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
 
Before any of this runs, the robot must be homed, stood up, and started from the web dashboard -- see `KNOWN_ISSUES.md` for that flow.
 
---
 
## Control Loop
 
`pi5_main.py` runs a ~100 Hz loop that fuses heading, chooses a steering command (manual, GPS, or the avoidance override), and rebuilds the gait only when the command or attitude actually changes. Everything that could block it is pushed onto its own thread:
 
| Thread | Job |
|---|---|
| Control loop | Steering decision, gait rebuild, Pico ack/recovery handling, status publish |
| Gait serial worker | The single writer to the four Pico UARTs -- gait frames, home/stop commands, recovery |
| `SensorHub` pollers | One each for IMU (50 Hz), GPS + compass (10 Hz), INA219 (1 Hz) |
| Vision worker | Depth inference + costmap, 1-3 fps; not started if the ONNX model isn't installed |
| Flask / ROS executor | Dashboard requests and ROS 2 callbacks |
 
The sensor split matters: those reads are synchronous I2C/UART transactions, and a NAKing or wedged bus blocks the kernel for its timeout. Inline, that used to stall steering and the IMU reflex with it. Now each poller owns its device and publishes a timestamped snapshot; the loop reads the snapshot with a staleness cutoff, so a dead sensor degrades to a safe default (flat attitude, hold last heading, skip the battery check) and a watchdog logs which poller is stuck. `python3 sensor_hub.py` runs a self-test with fake sensors, including a hung-bus case.
 
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
 
**Not yet verified on hardware** — see `KNOWN_ISSUES.md` for what needs measuring first.
 
---
 
## Dashboard
 
Open `http://<pi-ip>:5000`. Top to bottom: live camera feed, **Steering** (direction readout, LEFT / FWD / RIGHT, the −90…+90 slider), **Startup** (Home ×4 → STAND → GO → STOP), **Navigation** (collapsible GPS routing + obstacle avoidance), then **Debug** per-leg deactivation at the bottom. Each card's wordy explanation is tucked behind a small **i** button. STAND and GO stay disabled, with a warning banner, until all four legs are homed; status pills up top show homed count / standing / walking.
 
The **Navigation** card holds two collapsible sections, both closed on load. *GPS Waypoints & Routing* has the NAV MODE master toggle, a waypoint editor (one latitude box and one longitude box per point, ▲/▼ to reorder, × to remove, **+ Add point**, **Send route** — nothing reaches the robot until pressed), and a **Start / Pause / Stop** transport row greyed out by state: Start runs the route from the first point, Pause holds position without advancing, Stop returns to manual and rewinds. With no route, NAV MODE just marches in place; on reaching the last point the robot holds position. *Obstacle Avoidance* has the avoidance toggle and its live state readout.
 
The markup lives in one place — `dashboard_page.py`, a plain string with no imports — so `stream_server.py` (the real node) and `dashboard_preview.py` (a hardware-free mock for working on the UI) render the identical page. Styling is an APEX red/black theme over Bootstrap 5, and Bootstrap is **vendored** at `Code/Pi5/static/bootstrap.min.css` rather than pulled from a CDN, so the page is fully functional when the Pi is its own access point with no route to the internet.
 
---
 
## Getting Started
 
> Full setup instructions are a work in progress. The notes below are enough to get running.
 
### Pi 5 Requirements
 
```bash
pip install pyserial smbus2 flask opencv-python adafruit-circuitpython-bno08x
pip install onnxruntime numpy        # obstacle avoidance only; optional
```

Obstacle avoidance is optional at runtime — if `onnxruntime` or the model file is missing, the import is caught, the dashboard shows `AVOIDANCE: NO MODEL`, and everything else runs unchanged.

Hardware init is non-fatal: a missing IMU, dead GPS, unplugged camera or absent power monitor each leave that subsystem reporting *down* — an amber "running degraded" banner on the dashboard, and a line on the optional SH1106 OLED (`boot_display.py`, wired per `HARDWARE.md`) — rather than stopping the boot. Only the Flask dashboard itself is required. `pip install luma.oled` enables the OLED; without it the same progress goes to stdout.
 
ROS 2 (Humble or later) required for `pi5_main.py`. For testing without ROS, use `single_leg_test.py` — it has no ROS dependency, runs a single leg, and serves the camera stream.
 
### Running the single-leg test
 
```bash
cd Code/Pi5
python3 single_leg_test.py
```
 
Then open `http://<pi-ip>:5000` in a browser for the control panel and live camera feed.
 
### Running full production
 
```bash
cd Code/Pi5
source /opt/ros/humble/setup.bash
python3 pi5_main.py
```
 
### Pico Firmware
 
Flash each Pico with MicroPython, then copy the contents of `Code/Pico/` to the Pico filesystem. The main loop starts automatically on boot. **The legs do not move on their own** -- each Pico just holds its power-on position under light PID until it's homed. From the web dashboard: home each leg individually, then **Stand**, then **Go**. A **Stop** control cuts motor power without losing homing.
 
---
 
## Project Status
 
| Component | Status |
|---|---|
| IK / FK engine | Complete |
| Gait generation | Complete |
| Pico PID motor control | Complete |
| Pi-Pico serial protocol | Complete |
| IMU stabilization | Complete |
| GPS navigation | Complete |
| Camera streaming | Complete |
| Recovery path | Complete |
| Homing / Stand / Go / Stop dashboard workflow | Complete |
| ML obstacle avoidance | Built, needs hardware tuning |
| Current sensing foot detection | In progress |
| Mechanical build | In progress |
| Custom PCB | In progress |
| CAD files | In progress |
 
---
 
## Repo Structure
 
```
Code/
├── Pi5/                 # Raspberry Pi 5 code
└── Pico/                # RP2040 MicroPython firmware
```
 
---
 
## License
 
MIT
 
---
