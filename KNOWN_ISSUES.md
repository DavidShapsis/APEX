# Known Issues

Defects found during a full read of the control path that were **not** fixed, with
why they were deferred, plus the manual configuration steps the fixes introduced.
Ordered by how badly they bite.

Fixed items are not listed here — see git history.

**Verify the gait before touching hardware:**

```bash
cd Code/Pi5/inverse_kinematics
python quadruped_sim.py --report      # numeric verdict, no dependencies
python quadruped_sim.py               # 3D animation, needs matplotlib
```

`matplotlib` is not currently installed (`pip install matplotlib`), which also means
`gait_testing2.py` and `gait_testing3.py` cannot run as-is. `--report` works without
it.

---

## PLANNED — migrate ground-contact sensing from FSR to current sensing

Decided direction, not yet implemented. Ground contact (used only for the abort
check — a foot touching down early, mid-swing, triggers `handle_recovery`) currently
comes from four digital FSR pins in `Pico/fsr.py`, one per leg, read as a plain
GPIO high/low through a physical pull-down resistor: `fsrs = [FSR(16), FSR(17),
FSR(18), FSR(19))]`, and `any_touchdown = any(f.state for f in fsrs)` in
`pico_main.py`. The plan is to replace this with current sensing on the joint
motors instead — detecting a stall/current spike (the leg meeting unexpected
resistance) rather than a dedicated foot-mounted pressure sensor.

**Not designed yet — flagging what the migration actually has to answer, not
guessing at it:**
- *Where the current gets measured.* Per-joint (at each `JointController`'s H-bridge)
  gives the finest signal but needs a shunt + ADC per joint (12 readings across 4
  legs) that doesn't exist yet. The existing `INA219` in `power_monitor.py` reads
  total *pack* current only — one number for the whole robot, not per-leg, so it
  cannot answer "which foot touched down" on its own.
- *Stall vs. touchdown are not the same event.* A current spike means the leg met
  resistance — that's also true for hitting an obstacle mid-swing, binding at a
  joint limit, or simple stiction, not only "foot reached the ground." The FSR is
  specific to ground contact; current sensing is a broader "something stopped this
  joint" signal and will need its own logic to distinguish the cases (or the
  abort logic needs to become "the leg stalled," a real semantic change, not just a
  sensor swap).
- *Threshold tuning is different, not simpler.* FSR threshold is a fixed digital
  high/low against a known resistor. A current threshold has to sit above normal
  swing-phase motor current (which itself varies with `kp`/duty cycle, bench-tuning
  which is already an open item above) and below a stall — those two need to be far
  enough apart under real load to have a safe midpoint.

**Keep this in mind for other work, not just this entry:** any future change
touching `fsr.py`, the abort/`ABORTED` protocol, or `handle_recovery` should treat
FSR as the *current, temporary* mechanism, not a permanent architecture decision —
don't build more FSR-specific tooling or tuning on top of it without checking
whether this migration has since landed.

---

## Homing — a manual step you have to do, before every power-on

Every Pico's encoder starts counting from wherever the leg physically is when it
boots — the firmware has no way to know the real angle on its own. It currently
assumes `initial_angle=0`, but the IK's neutral standing pose is
**roll −15.55°, pitch 44.4°, knee 85.1°**, not zero. So the first gait frame used to
command ~45° and ~95° of travel from a wrong assumed position, at effectively full
PWM. **Resolved below — the whole flow (Home → Stand → Go → Stop) is built and
verified; one open question about mechanical confirmation remains at the end.**

**Manual, mechanical, not automated: before powering on, position each leg by hand**
into this reference pose, one leg at a time, and treat that as the zero point:

- **Hip roll = 90°** — abductor perpendicular to the body, pointing up.
- **Knee = 180°** — locked straight, thigh and shin colinear.
- **Hip pitch = 0°** — the leg (thigh+shin, straight) perpendicular to the body's
  lengthwise axis, no fore/aft lean.

This isn't a guess — it's derived from that description using the actual IK/FK code:
with the knee locked straight the thigh-shin triangle degenerates, which forces the
knee-side interior angle to exactly 0° regardless of segment length, which in turn
forces pitch = 0° to be the *exact* angle with zero fore/aft foot offset (verified:
`calculate_fk(90, 0, 180)` gives y = 0.0000 cm, and round-trips back through
`calculate()` to the same three angles). See `HOME_POSE` in `ik_and_gait.py`.

All four legs get commanded the identical standardized angles here — it's each
joint's `reverse` flag on the Pico that makes the same command swing each leg out to
its own correct physical side.

### RESOLVED — Home button, one leg at a time
How does the firmware know a leg has been placed and is ready to be zeroed? It
doesn't, on its own — the human says so, one leg at a time, from the web dashboard.
Each Pico recognizes a 2-byte `HOME_MARKER` (`0xAB 0xAB`, distinct from the gait
protocol's `0xAA 0xAA` start marker) that means "wherever I physically am right now
IS HOME_POSE" — it calls `zero_at()` on all three joints (new on `JointController`,
same formula as the `initial_angle` setup in `__init__`, just callable after boot),
then acks `HOMED,<id>\n`. **Nothing moves when this runs** — it only redefines what
the encoder count means.

The dashboard has a `Home FL` / `Home FR` / `Home RR` / `Home RL` button, one per
leg, exactly as planned ("each leg gets homed independently 1 by 1"). `pi5_main.py`
tracks `self.homed[leg_id]`, sourced only from the Pico's ack (not just "we sent the
command"), and `request_stand()` / `engage_walking()` both hard-refuse — server-side,
not just a greyed-out button — unless every leg reports homed.

### RESOLVED — boot no longer moves the legs
It used to ramp straight into the walking gait as soon as it powered up, which —
combined with the fact that nothing verified homing had actually happened — meant an
unhomed robot would still slam its legs to wherever the gait's first frame put them.
Boot now only runs leg identification and prints a reminder; every Pico just holds
its power-on position (target 0,0,0) under light PID, which is what produces the
small holding jiggle rather than a real move. Motion only starts once the operator,
in order: homes all four legs, presses **Stand**, then **Go** (both below).

### RESOLVED — smooth ramps instead of one big jump: Stand and Go
Even with homing done correctly, going straight from HOME_POSE into the walking
gait's first step is itself a big move (**105° of roll, 44° of pitch, 95° of knee**
— see the table below). `ik_and_gait.cartesian_ramp()` interpolates a move like that
in a straight line in foot-position space over 40 steps; `pi5_main.build_ramp()`
wraps it for any fixed start pose → per-leg target. Verified: a ramp starts exactly
at its start pose, ends at its target, and the intermediate foot path is a straight
line to within 0.02 cm (rounding).

That one big jump is now split into two smaller, explicitly user-triggered moves
instead of a single automatic one at boot:

- **Stand** (`request_stand()`) ramps every leg from HOME_POSE into a static,
  feet-under-hips pose (`self.stand_pose`) — refuses unless every leg is homed.
- **Go** (`engage_walking()`) ramps from `self.stand_pose` into the walking gait's
  first frame, then starts the continuous cyclic walk — refuses unless the robot is
  already standing.

| joint | HOME_POSE | gait neutral stance | travel |
|-------|-----------|---------------------|--------|
| roll  | 90.0°  | −15.55° | −105.5° |
| pitch | 0.0°   | 44.44°  | +44.4°  |
| knee  | 180.0° | 85.11°  | −94.9°  |

(Table is the original single HOME_POSE → gait-neutral jump that motivated building
a ramp at all — the numbers above show why slamming straight there was the problem
in the first place. Stand and Go now cover that distance in two separately-gated
hops rather than one.)

Sending a one-shot ramp to all four legs at once (rather than one leg's recovery
path) needed a small worker change: the normal cyclic-gait path always phase-offsets
each leg's buffer index and always ends with the *cycle* marker. A one-shot,
multi-leg send needs neither — the four ramps are independent point-to-point paths,
not a shared cycle sampled at different phases, so applying the phase rotation would
have each leg read a different, wrong ramp entry (verified this directly: with the
real offsets applied, a nonzero-offset leg samples the wrong entry; with offsets
zeroed, every leg samples its own ramp correctly). `send_entire_gait()` takes a
`cycle` flag, and the worker picks offsets `[0,0,0,0]` and the one-shot marker when
`cycle=False`.

### RESOLVED — Stop
Every Pico recognizes a third 2-byte command, `RELAX_MARKER` (`0xAC 0xAC`), that cuts
PWM to zero on all three joints without touching `gait_buffer`, `current_targets`, or
the encoder ISR. Pressing **STOP** on the dashboard sends this to every identified
leg, and also clears `walking_enabled`/`standing` on the Pi so the cyclic gait stops
being re-sent. Power resumes automatically — no separate "resume" command — the
moment either a `HOME` command or a complete gait/ramp frame arrives, which is what
pressing **Home** or **Stand** again naturally does.

**Read this before using it:** Stop does not lower the robot into any kind of
resting pose first. If the robot is standing or walking under its own weight when
you press it, cutting power will let the legs sag or the robot collapse the instant
torque drops — there is no sit-down sequence. It's built for "the robot is already
off the ground or otherwise supported and I want to relax the motors without losing
homing," not as a safe way to halt a walking robot mid-stride.

### Bug caught while building Stand/Go, not previously exercised
`cartesian_ramp()` unpacked its target with `calculate_fk(*target_angles)`. A
gait-frame entry is `[roll, pitch, knee, is_swing]` — 4 elements — so this raised
`TypeError` the moment it ran against a real gait frame instead of the 3-element
tuples the one existing test used. It would have crashed the first `engage_walking()`
call (and, before that, the original boot-time ramp) the first time it actually ran,
on hardware, with `rclpy` installed. Fixed by slicing to `[:3]` before unpacking.

### Still open, and worth deciding before any of this is trusted
Homing has no mechanical confirmation at all — if a leg is 10° off from the real
pose when you press its button, the firmware has no way to notice. It's a straight
trust-the-human step, just now a button press instead of "power on and hope." A hard
stop or limit switch would close this gap; not built here since it needs a decision
about the mechanism.

---

## Before you walk it

### Set LEG_ID on each Pico
`pico_main.py` has a `LEG_ID` constant at the top. Each of the four boards needs a
**different** value matching the corner it is bolted to:

| `LEG_ID` | Corner |
|----------|--------|
| `0` | Front Left |
| `1` | Front Right |
| `2` | Rear Right |
| `3` | Rear Left |

Each board announces `LEG,<id>` over UART every 500ms until it receives its first
gait frame, and the Pi builds the port→corner map from that — so UART wiring order
no longer matters. But two boards sharing an ID, or one set to the wrong corner,
produces a wrong gait phase and the robot falls. **The Pi logs the map it detects at
startup; check it before any powered walking.**

### Set the left/right `reverse` flags
The joint setup in `pico_main.py` takes a `reverse` argument per joint, currently
`False` everywhere. Left and right legs are mirror-image copies — the abductor
segment sticks outboard on both sides — and a reflection reverses handedness, so
one side needs its motor polarity flipped. The commanded angles are identical
either way; only the `reverse` flags differ. Manual per-board setting, like `LEG_ID`.

### Front/rear mirroring is handled in software — note which pair
The IK solves a **knee-forward** leg: at neutral stance the knee node sits at local
y = +18.8 cm. That matches how the **rear** pair is mounted. The **front** pair is
bolted on turned round so its knees point back (as on a real dog), so the front legs
are the ones whose stride gets mirrored — `mirror_y=(leg_id in FRONT_LEGS)` in
`pi5_main.build_gait()`.

It is a spatial mirror, **not** a time reversal. Reversing the cycle would also shift
each flipped leg half a cycle and scramble the FL → RR → FR → RL crawl order.

### Body geometry (set — recorded here because it lives nowhere else)
`quadruped_sim.py` uses `BODY_LENGTH = 67.5`, `BODY_WIDTH = 53.0` cm. Both are
**hip-pivot spacings**, which is what the support polygon is built from — the shell
is 950mm end to end, but the hip pitch axes are only 675mm apart. Using the outer
length here would flatter the stability margin.

Same file has `LEG_FLIP_X` / `LEG_FLIP_Y`, modelling left-right mirroring and the
front pair's reversed mounting. `mirror_y` is derived from `LEG_FLIP_Y`, so the
mount model and the gait cannot silently disagree — but if your mechanical layout
differs from what those tables say, fix them or the sim will lie to you.

---

## Debug tool: per-leg deactivation

The dashboard has a `Deactivate <leg>` / `Reactivate <leg>` toggle per leg, for
bench testing. Deactivating a leg does **not** change the gait loop in any way —
`pi5_main.py` still computes that leg's target every tick exactly as if nothing
were deactivated — it only skips the wire write (open marker, per-step payload, and
close marker, all three) for that leg's Pico. That Pico is left completely alone:
no empty frame, no reset, it just keeps holding whatever it was last actually told.
The other three keep walking normally.

Does not touch `self.homed` — deactivating and reactivating a leg never requires
re-homing it. Intended for isolating one leg's Pico on the bench (or watching the
other three walk without one leg's motion muddying the picture), not for anything
load-bearing: a deactivated leg gets no updated commands at all, so if it was mid-air
when deactivated, it stays exactly there.

---

## Will make the robot fall over

### RESOLVED — body shift, phased with the lift sequence
The sim reported 100% "statically stable" but that was a binary in/out test hiding
the real number: the **margin** — distance from the body centre to the nearest edge
of the support triangle — had a worst case of **+0.42 cm** at 67.5 × 53 cm. The cause
is geometric: with feet directly under the hips, lifting one leg leaves a triangle
whose hypotenuse runs corner-to-corner *through* the body centre, on the edge by
construction. 4 mm gets consumed instantly by payload offset, the swinging leg's own
mass, or any ground unevenness.

`ik_and_gait.py` now has `body_shift_profile()`: it leans the body 2 cm toward the
diagonally-opposite hip from whichever leg is about to lift, one tick ahead of the
lift (during the four-feet-down beat), then leans back. `apply_body_shift()` folds
that into each leg's foot path in its own local frame. `pi5_main.build_gait()` calls
both; `quadruped_sim.py` calls the identical functions, so the two cannot disagree.

The first attempt used a box filter to smooth the profile and only reached +0.81 cm
— worse than expected. The filter was straddling the sign flip at each lift boundary,
which is also the instant the margin is thinnest, diluting the lean exactly when it
mattered. **No smoothing (`smooth_width=1`) is now the default**, verified against the
alternative — see the comment on `body_shift_profile` for the measured numbers.

Result: **worst-case margin is now +2.38 cm**, clearing the 2 cm bar, at a joint-rate
cost of 235 → 266 deg/s (65% → 74% of the motor's free speed — still 26% headroom).
`quadruped_sim.py --report` now shows `VERDICT: gait is viable`.

### RESOLVED — `height2` (stance push) set to 0
Three feet are planted at different points in the stance phase, so a nonzero
`height2` commanded them to depths spanning 2.17 cm — on flat rigid ground they
cannot all be there, and it becomes body bob or lost contact. `STANCE_PUSH` is now
`0.0` in `pi5_main.py`, `quadruped_sim.py`, and `single_leg_test.py`. Downward reach
for uneven terrain would need closed-loop FSR contact to do properly, not a fixed
per-step push — left for later, deliberately not brought back as a blind default.

### RESOLVED — `STEP_TICK_MS` raised 20 → 40 ms
Kept here because the tradeoff is worth understanding before anyone lowers it again.
40 ms gives a peak of 235 deg/s (65% of free speed) at 16.7 cm/s. The simulator now
checks this every run and fails if the commanded rate exceeds the motor.

Peak commanded joint rate, per phase, at the old 20 ms tick:

| phase | roll | pitch | knee |
|-------|------|-------|------|
| stance | 11 dps | 91 dps | 69 dps |
| **liftoff** | 71 dps | 253 dps | **469 dps** |
| swing | 51 dps | 345 dps | 299 dps |
| **touchdown** | 71 dps | 163 dps | **469 dps** |

The goBILDA 5302 at 99.5:1 / 60 RPM gives **360 deg/s at the output shaft, free
running**. The gait asks for **1.30x that** at liftoff and touchdown, and under the
load of a real quadruped nothing like 60 RPM is available. The PID will simply
saturate and the leg will lag its target — the foot lands late and in the wrong place.

The peaks are at the phase boundaries, not mid-swing: the foot reverses direction
there, and with only 4 swing samples that corner is sharp.

| `STEP_TICK_MS` | cycle | speed | peak rate | |
|----------------|-------|-------|-----------|---|
| 20 (current) | 0.40 s | 33.3 cm/s | 469 dps | too fast |
| 25 | 0.50 s | 26.7 cm/s | 375 dps | too fast |
| 30 | 0.60 s | 22.2 cm/s | 313 dps | 87% of free speed — still tight |
| **40** | 0.80 s | 16.7 cm/s | 235 dps | 65% — realistic under load |
| 50 | 1.00 s | 13.3 cm/s | 188 dps | conservative |

Not changed, because the right value depends on what the motors actually deliver
under load, which is a bench measurement rather than something derivable. **Start at
40 ms** and work down while watching whether the legs track their targets.

Softening the corner would also help — more swing samples, or easing the velocity
into liftoff/touchdown rather than starting the arc at full horizontal speed.

Note the same argument in Y is already handled — stance `y` is deliberately linear in
time so all planted feet travel at identical speed. An elliptical stance would make
them differ by up to 0.85 cm/tick (127% of a step) and scrub against the ground.

---

## Will misbehave

### `current_state` conflates operating mode with transient activity
`pi5_main.py` — `RobotState` is used both for MANUAL/AUTONOMOUS and for RECOVERY.
Three separate bugs follow:

1. After any recovery the worker hard-sets `current_state = MANUAL`, so a stumble
   silently drops the robot out of GPS nav while the web UI still shows NAV: ON.
2. `nav_mode_callback` overwrites `current_state` unconditionally from the executor
   thread. If a toggle lands while `handle_recovery` has staged a recovery, the
   worker's `state_check == RECOVERY` test fails and **the recovery never
   transmits** — the leg stays frozen with `has_aborted = True`.
3. The worker's state check races that same callback.

Fix is `self.mode` + `self.activity` as separate fields. Deferred because it changes
control flow rather than fixing a localized defect.

### RESOLVED — one leg recovers while the other three keep walking: policy is "go to neutral"
`handle_recovery` used to send the recovery path only to `trigger_serial`, leaving
the other three legs cycling their existing walking buffer — trying to keep walking
on three legs while the fourth recovers. Policy decided: **go to neutral**, not
freeze in place. The other three are now sent, in the same recovery pass, a direct
one-shot move to `self.stand_pose` (the same static point the aborting leg's own
`get_recovery_gait` already targets) — so all four legs converge on the same
feet-under-hips pose. `standing`/`walking_enabled` are cleared when recovery starts
and `standing` is set once it completes, so **Go** is required to resume walking
rather than the robot silently continuing on whatever it was doing.

**Caveat, not fully solved:** the other three legs have no live position feedback —
the Pi only learns a leg's actual angle when *that* leg is the one that aborts and
reports it. So unlike the Home→Stand→Go ramps, this can't be a smooth Cartesian
interpolation from a known start; it's a direct PID move to `stand_pose`, bounded by
the same PWM limits as any ordinary gait step. Walking-gait poses stay close to
`stand_pose` by construction (that's the whole point of the stand height), so the
jump should be small, but it hasn't been measured on hardware.

### No stop command mid-turn, and BACK was removed rather than fixed
```python
steering_factor = chosen_direction / 45.0
steering_factor = max(-1.0, min(1.0, steering_factor))
```
`direction = 0` means *walk forward* — there is no zero-velocity travel-direction
command in the production UI (`single_leg_test.py` has a STOP button for this;
production doesn't). Note this is different from the new **STOP** button (see the
Homing section at the top), which cuts motor power entirely rather than commanding
zero velocity — there still isn't a "keep standing, walk forward at 0 speed" state.
The **BACK** button (which used to send 180°, saturating identically to RIGHT's 90°
— it was a hard right turn, not reverse) has been **removed** from the dashboard
rather than fixed, since a working reverse needs real omnidirectional steering (see
`direction_angle` below), not a quick patch. The slider still lets you drag to 180°
and get the same silent right-turn saturation — that underlying steering-math issue
is unchanged, just no longer surfaced as a labeled button.

Deeper cause: `chosen_direction` means "travel direction" in manual mode but
"heading error" in autonomous mode — two different quantities sharing one variable.
Fixing properly means separating turn-rate from travel-direction commands, and
adding an actual zero-velocity state the gait can return to (distinct from STOP's
full de-power).

Related: `GaitPath`'s `direction_angle` parameter — the omnidirectional steering
mechanism — is hardcoded to `0` at every call site in `pi5_main.py`, so differential
stride is the only steering actually wired up.

### Mission end doesn't stop
`Navigator.calculate_nav` returns `None` once waypoints are exhausted, so
`chosen_direction` falls through to the last manual direction and the robot walks
forever. Blocked on having a stop command.

### RESOLVED — IMU stabilization is now differential, not a common offset
It used to feed pitch into `center_stride_y` and roll into `lateral_roll_offset`,
applying **the same offset to all four legs**. Measured directly: at the maximum
correction values, the change in every foot's Z was **0.0000 cm** — a body
*translation* with zero restoring moment, so no gain would ever have made it work.

`ik_and_gait.attitude_height_offsets(roll_deg, pitch_deg)` now computes a **per-leg**
height change from each hip's position: `dz = gain * (hx*tan(roll) - hy*tan(pitch))`.
Raising the low corners and lowering the high ones is what produces an actual
restoring moment. `build_gait()` feeds this into `center_height_z` per leg. Verified
in the simulator: 15° of simulated roll now produces a measurable left/right foot
height difference; 15° of pitch produces a front/rear difference; zero tilt gives
identical heights across all four legs (as it must).

`gain = 0.6` (damped, not a full correction in one update — avoids overshoot) and
`limit_cm = 4.0` (caps how far one corner can be driven from stand height) are in
`pi5_main.py`; both are guesses that need bench tuning against the real BNO085.
`max_tilt_deg = 30` in the function itself clamps the input before it reaches
`tan()`, which blows up near 90°.

Sign convention: positive roll is taken as right-side-down, positive pitch as
nose-up. **If the BNO085 is mounted with either axis flipped relative to that, the
correction will actively tip the robot the wrong way — check this on the bench
before trusting it, ideally with the robot held up off the ground first.**

### Compass has no declination or tilt compensation
`get_heading()` is a raw two-axis `atan2(y, x)` — magnetic, uncalibrated for
hard/soft iron, uncompensated for tilt. GPS bearings are *true* north; declination
at the current waypoints (~41.05N, -74.14W) is about **-12°**. A platform that
pitches and rolls by design also makes an uncompensated 2-axis heading very noisy.
`get_heading()` additionally returns `0.0` on exception, indistinguishable from an
actual north heading.

### IK clamps unreachable targets silently
`InverseKinematics.calculate` clamps out-of-range law-of-cosines arguments and the
shoulder-to-foot distance instead of reporting that a target cannot be reached. Ask
for z = 55 cm and the foot lands at 52.11 with no exception and no flag, so the
caller cannot tell the difference between "done" and "as close as I could get".

Harmless for the current gait (it uses z = 31–38.5 cm, well inside the 2.47–51.21 cm
annulus) but it will hide mistakes in any future terrain or body-shift work. A
`reachable` flag on the result would fix it; not added because nothing reads it yet.

### PID: `kp` still needs bench tuning (windup ruled out, `kd` fixed)
Simulated against a motor model (360 deg/s free speed, first-order response,
stiction, and real encoder quantisation), running the actual `JointController` code:

- **No windup.** The I term peaks at 0.2–2.2 against its clamp of 50 — 0.4–4%. The
  deadband zeroes it whenever |error| < 1°, and the error changes sign every cycle,
  so it never accumulates. Not a risk at any motor time constant tested.
- **`kd = 0.05` was provably wrong and is now 0.** The encoder resolves 0.129°, so at
  the ~2 ms loop rate a single count reads as 64.6 deg/s — times `kd = 0.05` that is
  **3.23 of output**, past full duty from one tick of quantisation. Removing it cut
  direction reversals from ~260/s to ~12/s and time at full duty from 58% to 20%,
  and improved tracking as well. Reinstate only with a filtered derivative.
- **`kp = 0.8` remains aggressive.** It saturates for any error above 1.25°, so with
  the 1.0° deadband there is only a 0.25° sliver of proportional control. In
  simulation `kp = 0.30` tracked better (stance mean error 0.82° vs 2.33°), but the
  model has **no gravity or load torque**, which is exactly what `kp` fights — a
  softer gain could sag on the real robot. Left alone deliberately; tune on the bench.

Residual tracking error at 40 ms/step, `kp` 0.30 / `kd` 0 in simulation:

| | stance | swing |
|---|--------|-------|
| max | 7.85° | 9.40° |
| mean | 0.82° | 2.78° |

Stance mean of 0.82° is ~0.35 cm of foot error, which is fine. The maxima are at
liftoff and touchdown, where the target reverses direction; 9.4° is ~4 cm at the
foot, against 5 cm of commanded swing clearance — **verify the foot actually clears
the ground on the bench.** Easing the velocity through those corners would help more
than gain tuning.

### Minor: stale target for one tick
`pico_main.py` resets `current_step_index = 0` when a frame completes, but
`current_targets` isn't refreshed until the next `STEP_TICK_MS` tick (40ms) — so the
leg drives toward the *previous* frame's last target for up to 40ms. Also means step
0 of every buffer is skipped, since the first tick advances to index 1. The
recovery "go to neutral" move for the other three legs (a 2-entry one-shot buffer
holding `stand_pose` twice) relies on exactly this behavior to settle onto its
target within one tick — harmless there since both entries are identical, but worth
knowing this mechanism is now load-bearing elsewhere, not just a latent quirk.
`HOME_MARKER` is unaffected — it sets `current_targets` directly, bypassing the
buffer entirely.

---

## Verify on hardware

### BNO085 constant names — check this first
`imu.py`:
```python
self.bno.enable_feature(BNO08X.REPORT_LINEAR_ACCELERATION)
self.bno.enable_feature(BNO08X.REPORT_ROTATION_VECTOR)
```
The Adafruit library exposes these as **module-level** constants named
`BNO_REPORT_LINEAR_ACCELERATION` / `BNO_REPORT_ROTATION_VECTOR`, not as class
attributes. If that's the case in the installed version this is an `AttributeError`
in the constructor, and `IMU(...)` at `pi5_main.py` has no try/except around it —
**the robot won't boot.** One-line check on the Pi.

### RESOLVED — BTS7960 dual enable confirmed tied together on the PCB
`JointController` drives a single `en_pin`; the chip has separate R_EN and L_EN (as
`BTS7960_Test.py` correctly does), which would have been a problem if they were
wired independently — one half of the bridge could stay disabled while the code
thinks the joint is live. **Confirmed on the real board: R_EN and L_EN are tied
together**, so a single `en_pin` is correct and nothing needs to change here.

Separate, still true: the abort path only zeroes PWM and leaves the bridge enabled —
there's no hard disable on fault. Not blocking, just worth knowing if a fault needs
to cut power at the driver rather than just at the PWM signal.

### Encoder zero must match HOME_POSE — this is what homing (top of doc) is for
Every joint's mechanical zero needs to be set at `HOME_POSE` (roll 90°, pitch 0°,
knee 180°) via the manual Home-button procedure at the top of this document, not at
the IK's neutral standing pose. If it's off, `calculate_fk` — used by the Stand/Go
ramps (`build_ramp()`) and by the recovery path alike — reconstructs the wrong
Cartesian position from reported joint angles, and all of them compute a path to the
wrong place. Nothing in the firmware currently verifies homing was done correctly
before Stand/Go accept it (see "Still open" in the Homing section).

### Dead branch in `imu.py`
The `bus_id is None` path calls `busio.I2C(scl_pin, sda_pin, ...)` with the string
pin names `"D1"` / `"D0"` rather than board pin objects. It would fail if used;
currently `bus_id=13` is always passed, so the branch is dead.

---

## Protocol gaps (accepted for now)

The Pi→Pico link has **no length field, no checksum, no sequence number, and no
ACK**. A corrupted frame produces wrong motion silently, and the Pi has no way to
know a leg is desynced.

Current mitigations, all added rather than the protocol being redesigned:
- byte-at-a-time start-marker scanning, so odd-byte misalignment recovers
- per-payload range validation, so non-finite/garbage angles are dropped
- `MAX_GAIT_STEPS` cap, so a straddled terminator can't grow the buffer to MemoryError

A frame truncated **mid-payload** can still desync the parser for up to a few frames
before the cap forces a rescan. The Pi no longer truncates its own frames, so this
should not arise in normal operation.
