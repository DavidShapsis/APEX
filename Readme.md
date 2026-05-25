 **⚠️ This is an ongoing project. The code, CAD, and documentation are all actively being developed and are not final. Expect frequent changes.**
 
---
 
# APEX — Autonomous Precision Exploration
 
A fully custom-built quadruped robot dog designed and coded from scratch by a 9th grader. No kit. No tutorial. Just CAD, math, and a lot of broken parts.
 
APEX is designed to walk across varied outdoor terrain using real-time inverse kinematics, stay balanced using an IMU, navigate to GPS waypoints autonomously, and stream live camera footage back to any device over WiFi. The long-term goal is onboard ML-based obstacle avoidance for fully autonomous terrain exploration.
 
![Status](https://img.shields.io/badge/status-in%20progress-yellow)
![Platform](https://img.shields.io/badge/brain-Raspberry%20Pi%205-red)
![Firmware](https://img.shields.io/badge/legs-RP2040%20%C3%974-blue)
![License](https://img.shields.io/badge/license-MIT-green)
 
---
 
## Features
 
- **Inverse Kinematics** — custom 3-link IK engine with forward kinematics for recovery, computing joint angles in real time for all four legs
- **Differential Gait Control** — tank-style differential steering with independent left/right stride lengths for smooth turning
- **IMU Stabilization** — BNO085 quaternion-based roll and pitch correction applied continuously to the gait path
- **GPS Navigation** — autonomous waypoint following using bearing and distance calculations from a HGLRC M100 GPS module
- **Live Video Streaming** — USB webcam feed served over Flask to any device on the same network
- **FSR Foot Sensing** — force sensitive resistors on each foot trigger an automatic recovery routine on unexpected ground contact
- **ROS 2 Integration** — inter-node communication via ROS 2 topics for direction commands and navigation mode switching
---
 
## Hardware
 
| Component | Qty | Notes |
|---|---|---|
| Raspberry Pi 5 | 1 | Main brain |
| Raspberry Pi Pico (RP2040) | 4 | One per leg, runs MicroPython |
| GoBilda 5302 Yellow Jacket Motor (99.5:1, 60 RPM) | 12 | 3 per leg |
| BTS7960 43A H-Bridge | 12 | One per joint |
| BNO085 IMU | 1 | Quaternion-based orientation |
| HGLRC M100-5883 GPS/Compass | 1 | Outdoor autonomous nav |
| INA219 Voltage/Current Monitor | 1 | Battery telemetry |
| Force Sensitive Resistors | 8 | Foot contact detection |
| Carbon Fiber Tube (16x12mm) | — | Lower leg structure |
| Aluminum 6063 Tube (1in OD) | — | Upper leg structure |
| 3S 11.1V LiPo 80C 5Ah | 1 | Motor power |
| 2S 7.6V LiHV 3.5Ah | 1 | Electronics power |
| Custom PCB | 1 | High-current motor control, in progress |
 
---
 
## Software Architecture
 
```
Pi 5 (ROS 2)
├── pi5_main.py          # Main control loop, IMU, GPS, gait generation
├── inverse_kinematics/
│   └── ik_and_gait.py   # IK, FK, GaitPath, GaitIK, RecoveryPath
├── imu.py               # BNO085 quaternion → roll/pitch
├── navigation.py        # GPS parsing, compass, waypoint navigation
├── stream_server.py     # Flask + OpenCV camera stream
├── webcam.py            # USB camera capture
├── power_monitor.py     # INA219 voltage/current
├── audio.py             # Bluetooth speaker alerts
└── single_leg_test.py   # Standalone single-leg test harness (no ROS/IMU/GPS)
 
Pico (MicroPython, x4)
├── pico_main.py         # UART receiver, gait buffer, PID execution loop
├── motor_control.py     # BTS7960 PID joint controller with encoder feedback
└── fsr.py               # Force sensitive resistor foot contact
```
 
### Pi to Pico Protocol
 
The Pi sends gait data over UART to each Pico using a binary protocol:
 
- **Start:** `0xAA 0xAA`
- **Payload:** 20 steps × 16 bytes each (`struct.pack('ffff', roll, pitch, knee, is_swing)`)
- **End:** `0xFF × 16`
Each Pico independently steps through the gait buffer at 20ms per step. The four legs are phase-offset by `[0, N/2, 3N/4, N/4]` for a trot gait pattern.
 
---
 
## Kinematics
 
The IK engine uses a 3-link chain (hip abductor, thigh, shin) solving for roll, pitch, and knee angles given a target foot position in (X, Y, Z):
 
1. **Roll** — solved in the X-Z plane using `atan2` + `acos` geometry on the abductor link
2. **Pitch/Knee** — solved in the virtual leg plane using law of cosines
Forward kinematics is used for the recovery path, reconstructing foot position from joint angles to interpolate back to home stance.
 
Segment lengths (cm): `a = 9.65` (abductor), `b = 26.84` (thigh), `c = 24.37` (shin)
 
---
 
## Gait
 
The gait path is a 20-step elliptical cycle parameterized by `theta ∈ [0, 2π]`:
 
- **Swing phase** (`sin(θ) > 0.1`): foot rises to 5cm above neutral height
- **Stance phase** (`sin(θ) ≤ 0.1`): foot extends 2.5cm below neutral height, pushing off the ground
Steering uses differential stride length between left and right leg pairs, similar to tank drive.
 
---
 
## Getting Started
 
> Full setup instructions are a work in progress. The notes below are enough to get running.
 
### Pi 5 Requirements
 
```bash
pip install pyserial smbus2 flask opencv-python adafruit-circuitpython-bno08x
```
 
ROS 2 (Humble or later) required for `pi5_main.py`. For testing without ROS, use `single_leg_test.py` — it has no ROS dependency, runs a single leg, and serves the camera stream.
 
### Running the single-leg test
 
```bash
cd Code/V2/Pi5
python3 single_leg_test.py
```
 
Then open `http://<pi-ip>:5000` in a browser for the control panel and live camera feed.
 
### Running full production
 
```bash
cd Code/V2/Pi5
source /opt/ros/humble/setup.bash
python3 pi5_main.py
```
 
### Pico Firmware
 
Flash each Pico with MicroPython, then copy the contents of `Code/V2/Pico/` to the Pico filesystem. The main loop starts automatically on boot.
 
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
| Mechanical build | In progress |
| Custom PCB | In progress |
| CAD files | In progress |
| ML obstacle avoidance | Planned |
 
---
 
## Repo Structure
 
```
Code/
├── V2/                  # Current version
│   ├── Pi5/             # Raspberry Pi 5 code
│   └── Pico/            # RP2040 MicroPython firmware
└── V1/                  # Archive
```
 
---
 
## License
 
MIT
 
---