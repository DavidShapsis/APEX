# Known Issues

Defects found during a full read of the control path that were **not** fixed, with
why they were deferred, plus the manual configuration steps the fixes introduced.
Ordered by how badly they bite.

Fixed items are not listed here — see git history.

> **⚠ RE-FLASH THE PICOS BEFORE THE NEXT POWERED TEST.** The 2026-08-29 review
> found two firmware defects, both fixed in `Code/Pico/pico_main.py`: the Stand
> and Go ramps were being silently discarded (the robot could not stand at all),
> and every gait re-send restarted the cycle from step 0, slamming the feet. A Pi
> running the new code against old firmware still cannot stand. Both are written
> up in the first two sections below.

**Verify the whole control path without hardware:**

```bash
cd Code/Pi5/inverse_kinematics && python quadruped_sim.py --report   # gait verdict
cd Code/Pi5 && python vision_obstacle.py                             # avoidance planner
cd Code/Pi5 && python sensor_hub.py                                  # sensor pollers
```

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

## RESOLVED — the Pico threw away every Stand and Go ramp (robot could not stand)

**This one stopped the robot working at all, and needs a Pico reflash to fix.**

`pico_main.MAX_GAIT_STEPS` was **32**. It exists as a desync guard — an overlong
frame means the byte stream lost sync mid-payload, and the cap stops the buffer
growing until `MemoryError`:

```python
gait_buffer.append(parts)
if len(gait_buffer) > MAX_GAIT_STEPS:
    gait_buffer = []          # the WHOLE frame is thrown away
    is_receiving = False
```

But the largest frame the Pi legitimately sends is the Stand/Go Cartesian ramp,
and `build_ramp(steps=40)` → `cartesian_ramp` → `range(steps + 1)` = **41
frames**. Every STAND and every GO therefore hit the guard at frame 33, had the
whole buffer discarded, and the remaining 8 frames plus the end marker were
rescanned as raw bytes looking for a start marker. The leg never left its homed
pose. Nothing logged it — from the Pi's side the frame went out fine.

The 20-frame walking gait, the 21-frame recovery path and the 2-frame
`settle_to_stand` were all under the cap, which is why the bug hid: everything
except the two ramps worked.

**Fixed** by raising `MAX_GAIT_STEPS` to **48** (still a desync guard, now above
every legitimate frame) and adding `PICO_MAX_FRAMES = 48` on the Pi, which
`send_entire_gait()` checks — it now logs and returns `False` rather than
believing an oversized frame was sent, and `request_stand` / `engage_walking` /
`settle_to_stand` propagate that failure instead of setting `standing = True`
regardless. Verified by replaying the real wire bytes through a copy of the
Pico's parser: the 41-frame ramp is accepted and ends holding the stand pose.

---

## RESOLVED — every gait re-send teleported the feet (25.6° / 17.6 cm, full duty)

Also needs the Pico reflash. The Pi rebuilds and re-sends the entire gait
whenever the steering command moves >5°, the IMU tilt moves >1.5°, or the
avoidance stride changes — which in normal walking is often. The Pico's
end-marker handler did:

```python
current_step_index = 0      # restart the cycle
```

So a foot that was mid-stance was told, in one 40 ms tick, to be at the start of
swing instead. That is a phase jump, not a geometry change, and it is large:

| re-send trigger | old (restart at 0) | now (phase kept) |
|---|---|---|
| straight → hard turn | 25.6° / 17.6 cm | **7.1° / 4.2 cm** |
| straight → 45° arc | 20.5° / 12.8 cm | **7.6° / 4.6 cm** |
| walk → march in place | 14.5° / 5.0 cm | **8.9° / 5.0 cm** |
| IMU tilt nudge only | 21.5° / 13.1 cm | **0.0° / 0.00 cm** |

At `kp = 0.8` (saturating above 1.25°) all of the left column is a full-duty slam
— the same failure class as the STAND lurch below, but happening several times a
second while walking rather than once on a button press.

**Fixed** by keeping the buffer index across a re-send *of the same walking
cycle* — same length, both cyclic — and resetting to 0 only for a genuinely new
trajectory (a one-shot ramp, or a cycle of a different length, which must run
from its first entry). `last_step_time` is still reset on every frame, which is
worth keeping: all four boards get their end marker within a few ms of each
other, so it re-aligns the 40 ms tick across legs and stops them free-running
apart. Verified in the wire simulation: all four legs keep the same index across
a re-send and exactly one leg is still airborne immediately after.

Note this is also *safer* for phase than the old behaviour. Previously, if one
leg missed a frame, the three that received it reset to 0 while the fourth kept
counting — a real desync. Now a leg that misses a frame simply keeps its place.

---

## RESOLVED — STAND while walking commanded a 68.6 cm lurch

Found during a full-codebase review, not previously exercised. `request_stand()`
always ramped from `HOME_POSE`, and **nothing anywhere refused it while the robot
was walking** — not the dashboard (`standBtn` was gated only on all-legs-homed),
not the `/stand` route, not the controller. Pressing STAND mid-walk therefore
commanded step 0 of a HOME_POSE ramp — roll 90°, knee 180°, legs straight out —
while the legs were actually near the gait pose:

| | roll | pitch | knee | foot |
|---|------|-------|------|------|
| walking gait pose | −15.55° | +44.44° | 85.11° | — |
| what STAND commanded | +90.00° | 0.00° | 180.00° | — |
| instantaneous error | **+105.6°** | −44.4° | +94.9° | **68.6 cm** |

`kp = 0.8` saturates above 1.25° of error, so that is full duty on every joint:
precisely the slam the whole Home → Stand → Go sequence exists to prevent.

**Fixed by making STAND mean the right thing in both states.** From rest it is
still the Cartesian ramp out of HOME_POSE. While walking it now calls the new
`settle_to_stand()`, a direct one-shot move of all four legs to `stand_pose` —
the same 2-entry buffer `handle_recovery` already sends the non-aborting legs,
and accepted there for the same reason. Measured worst case from *any* walking
gait pose (15 cm steering stride, body shift on) is **16.3° at a joint / 9.2 cm
at the foot**, against 105.6° / 68.6 cm before.

Side effect worth knowing: this is the first way to halt a robot that is on its
feet without dropping it. STOP cuts holding torque and lets it sag; STAND now
stops the walk and holds the stance powered. The dashboard says so.

Not a full fix for the missing zero-velocity state (see "No stop command
mid-turn" below) — there is still no "keep walking forward at 0 speed".

---

## RESOLVED — one InverseKinematics shared across threads returned wrong angles

`calculate()` stored its answer on the engine (`self.roll/pitch/knee`) and
returned `self`, so the caller read the angles off shared mutable state *after*
the call returned. `pi5_main` has a single `self.ik_engine`, used from:

- the main control loop — `build_gait()` → `GaitIK` → `calculate()`
- ROS executor threads — `stand_callback`/`go_callback` → `build_ramp()` →
  `cartesian_ramp()` → `calculate()`

Those genuinely overlap (the STAND-while-walking path above is exactly such a
case), and when they do, one thread reads the other thread's joint targets —
which then go straight to a motor. Measured with the GIL switch interval forced
low: **60% of solves corrupted** (24066 of 40000). At the default 5 ms switch
interval it is rare rather than impossible, which is the worst kind of bug.

**Fixed:** `calculate()` now returns an `IKSolution` — a small `__slots__`
object carrying its own roll/pitch/knee — and every intermediate in the solve is
a local. The engine holds only the segment lengths, which are read-only after
construction, so it is safe to share. Attribute names are unchanged, so all four
call sites needed no edit. Re-measured after the fix: **0 of 40000 corrupted.**

Related and also fixed: `request_stand()` and `engage_walking()` were writing
`self.standing` / `self.walking_enabled` without holding `serial_lock`, while
`publish_homing_status()` reads them under it.

---

## Swing clearance is 4.76 cm, not the nominal 5.0

`SWING_HEIGHT = 5.0` is the *continuous* peak of `h1·sin(πs)`, but the swing is
sampled at only 5 points — `s = 0, 0.2, 0.4, 0.6, 0.8` — which never lands on
the `s = 0.5` peak. The highest commanded lift is `5.0·sin(0.4π) = 4.755 cm`.

This matters because it appears in a safety argument: the PID tuning section
below reasons that 9.4° of peak tracking error is "~4 cm at the foot, against
5 cm of commanded swing clearance". The real figure is **4.76 cm**, so the
margin is 0.76 cm rather than 1.0 — about 25% thinner than stated. Still
positive, still needs the bench check that section already asks for.

Raising the swing sample count (a larger `num_steps`, or a finer swing fraction)
would both recover the nominal clearance and soften the liftoff/touchdown corner
that drives the peak joint rate — see `STEP_TICK_MS` below, which wants the same
thing for a different reason.

---

## Not a defect: stance y quantisation

Recorded because it looks like one. `GaitPath.generate_path` rounds each
coordinate to 2 dp, so consecutive stance steps differ by −0.66 or −0.67 cm
rather than a constant −0.667. The underlying formula is exactly linear
(verified: unrounded deltas are a single value); the variation is 0.005 cm of
rounding, 0.05 mm at the foot. The elliptical-stance scrub the design
deliberately rejects is 0.85 cm/tick — **170× larger**. Nothing to do here.

---

## Obstacle avoidance — built and simulated, never run on hardware

`vision_obstacle.py` plus the dashboard toggle. The planner logic is verified
(`python vision_obstacle.py` runs a self-test; a closed-loop sim against the real
`Navigator` showed obstacles struck by 0.95 m with avoidance off cleared by
0.25–0.50 m with it on, no oscillation). **None of the perception side has been
checked against a real camera on the real robot**, and several numbers are
educated guesses until it is.

### Tune these on the bench before trusting it outdoors

- **`ROI_TOP` / `ROI_BOTTOM` (0.45 / 0.95).** Which band of the frame counts as
  "the ground ahead". Depends entirely on where the camera is physically mounted
  and at what angle — a guess until checked against a real still. Too high and it
  reads the sky as clear; too low and it stares at the robot's own nose. Turn
  avoidance on and look at the overlay on the dashboard feed.
- **`OBSTACLE_MARGIN` (0.12).** How much nearer than its own row's ground a pixel
  must read before it counts. Measured insensitive between 0.05 and 0.25 on the
  synthetic scenes, so it is not a knife edge, but it has never met real terrain
  texture. Raise it if grass or gravel trips it, lower it if low obstacles slip
  through. (This replaced an absolute `NEAR_THRESH` — see below.)
- **`FLAT_GRADIENT_MIN` (0.10).** Only fires for a column with no near/far
  contrast at all, i.e. something filling the view. A camera angled further down
  sees less depth range in the band, which shrinks every column's gradient and
  makes false "blocked" more likely — check this once the mount is final.
  Raising it halts more readily, which is the safe direction.
- **`FOV_DEG` (60.0).** Assumed, not measured. Only scales bin index → steering
  angle, so an error here makes turns consistently too shallow or too sharp
  rather than breaking detection.
- **`CLEAR_HOLD_S` (20.0).** How long to drive straight after the obstacle leaves
  view before handing steering back to the navigator. This is the number that
  decides clearing vs. grazing — see the comment on it for the swept data. It is
  a time standing in for a distance because there is no odometry, so it is only
  correct at the assumed ~10 cm/s avoidance speed. **If `STEP_TICK_MS` or the
  base stride changes, this needs re-deriving.**

### RESOLVED — an absolute depth threshold read the ground as an obstacle

Found on a real capture: a person standing in the middle of an otherwise clear
path, with both sides plainly open, reported **fully blocked**. The depth model
was fine — the figure was cleanly segmented — the costmap was wrong.

For a forward-facing camera on flat ground, the distance to the ground at image
row `r` goes as `1/(r - horizon)`, so the inverse depth the model emits ramps
smoothly from far at the top of the frame to near at the bottom. The bottom of
the ROI is therefore *always* near: it is the ground half a metre in front of the
feet. `NEAR_THRESH = 0.55` flagged that ground everywhere. Reproduced on a
synthetic scene built from the flat-ground equation: **bare open ground scored
0.42 in every bin** against a 0.25 block threshold.

**Fixed** by making detection ground-relative: a pixel is an obstacle when it
reads `OBSTACLE_MARGIN` nearer than that row's own `GROUND_REF_PCT` percentile.
An upright object occupies rows that would otherwise show ground far away, so it
stands out sharply. The reference percentile sits below the median deliberately,
so a wide object cannot drag it up to its own depth and hide itself. A separate
per-bin flat-gradient test catches the one thing the relative test cannot see —
something filling a column top to bottom, leaving no contrast to find.

| scene | before | after |
|---|---|---|
| open ground | `#########` | `.........` |
| figure dead centre | `#########` | `....#....` |
| figure on the right | `#########` | `.......#.` |
| wall filling the view | `#########` | `#########` |

A useful property that fell out of it: because the reference comes from the row
itself, a **uniform slope shifts the whole row together and is not flagged**.
Only localised protrusions are, which is what you want on terrain.

These four scenes are now a regression test in `python vision_obstacle.py`, and
the tuning notebook imports the module rather than keeping its own copy of the
pipeline — the two had already diverged once.

### Known limits of the approach itself

- **Monocular depth is relative, not metric.** No true distances, and the scale
  shifts frame to frame. Fine for "something bulky is close", weak for judging
  gap widths.
- **Drop-offs and holes read as *farther* than the ground, never nearer, so the
  detector cannot flag them at all.** This is inherent to the method, not a
  tuning problem: the test asks "is anything closer than the ground should be",
  and a hole is the opposite. The FSR / `ABORTED` recovery path is the only
  backstop, and it only fires on contact.
- **Body width is handled by a fixed angle, but the correct angle depends on
  range.** What has to fit through a gap is ~72 cm (53 cm hip spacing plus the
  abductor sticking out ~9.65 cm each side — *not* the 53 cm `BODY_WIDTH_CM`).
  That subtends 71.7° at 0.5 m but only 20.5° at 2 m — 10.8 bins down to 3.1 —
  and the costmap has no depth axis to tell those apart, since the ROI band
  flattens roughly 0.5–2 m into one row of bins. `CORRIDOR_BINS=3` plus
  `INFLATE_BINS=1` enforces a flat 33.3°, which matches the true width only
  beyond **~1.2 m**; nearer than that it under-provisions on paper, and relies
  on a close obstacle filling enough bins to trip `BLOCKED` instead. Fixing this
  properly needs a range-resolved costmap (split the ROI into near/far rows and
  require a wider corridor in the near row), not a bigger constant — raising
  `CORRIDOR_BINS` to 5 demands 46.7° of a 60° view and leaves almost nothing
  passable.
- **Thin obstacles are unreliable.** Table legs and wires are close to the
  resolution limit at `INPUT_SIZE = 266` and may not survive the bin averaging.
- **No reverse.** Steering is a body yaw now (see the steering entry below), so
  the robot *can* spin in place — `ESCAPE` and a hard turn command both do — but
  it still cannot walk backward. A robot nose-first in a dead end can rotate to
  face out, which the old differential steering could not do, but if it is
  wedged it still needs a human.
- **`BLOCKED` halts by setting stride to 0**, which makes the feet lift and land
  in the same spot — it keeps standing and stays IMU-levelled. This is
  deliberately *not* the dashboard STOP, which cuts holding torque and would let
  a standing robot sag. Verified: `GaitPath` with `length=0` gives all feet
  `y = 0.0` with 4 airborne steps still in the cycle.
- **Fails open, not closed.** A stale costmap (camera died, worker stalled,
  older than `STALE_AFTER_S`) drops back to normal walking rather than halting.
  That is the right call for a slow robot with a human nearby, but it does mean
  a silently dead camera looks like "avoidance is on and the path is clear".
  Watch the dashboard state readout.
- **CPU contention is unmeasured.** Inference is capped at
  `cpu_count - 2` threads to leave room for the 100 Hz control loop and the four
  serial writers, but the effect on control-loop timing has not been measured on
  a loaded Pi 5.

### Validated against real ground-robot imagery

The tuning notebook's Test 1 now runs eight photographs taken from actual ground
robots — camera low, pointed straight ahead, ground plane filling the lower
frame — rather than scenic stills. Two results are worth carrying forward:

- **`INFLATE_BINS = 1` is load-bearing.** Dropping it to 0 makes an indoor
  corridor with a wall down each side report CLEAR. Confirmed on real imagery,
  not just simulation.
- **A trail that looks walkable reports BLOCKED, correctly.** Its clear gap
  measures 26.7°, which against the 72 cm body is 47 cm at 1 m, 71 cm at 1.5 m,
  95 cm at 2 m — genuinely too tight near, fine further out. A good illustration
  of the range-dependence limitation above: the costmap has no depth axis, so it
  cannot express "tight now, fine in a metre" and halts instead.
- **A low cardboard box on paving, several metres out, is missed.** Too few
  pixels clear `OBSTACLE_MARGIN` for its bin to reach `BLOCK_FRAC`. It would be
  seen on approach. Low obstacles at distance are the weak spot, with the
  `ABORTED` foot-contact path as the only backstop.

### First hardware session should be, in order

1. Robot on a stand, off the ground. Toggle avoidance on, watch the overlay, and
   tune `ROI_*` and `NEAR_THRESH` against real obstacles at real distances.
2. Check the reported inference time on the dashboard; confirm the control loop
   is not visibly stuttering.
3. Only then, on the ground, in manual mode, walk it at a single obstacle.
4. GPS/autonomous mode last.

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

Confirmed on re-review that a *duplicate* ID fails safe rather than dangerously:
the second board to claim an ID is logged as an error and never enters
`port_by_leg`, so `request_home_leg` refuses it, `all(self.homed.values())` never
becomes true, and STAND is refused. You get a robot that will not stand, not one
that walks on three legs. A *wrong but unique* ID does not fail safe — all four
home, and the gait phases go to the wrong corners. Check the logged map.

### Set the left/right `reverse` flags — now matters for turning, verify it
The joint setup in `pico_main.py` takes a `reverse` argument per joint, currently
`False` everywhere. Left and right legs are mirror-image copies — the abductor
segment sticks outboard on both sides — and a reflection reverses handedness, so
one side needs its motor polarity flipped. Manual per-board setting, like `LEG_ID`.

**"The commanded angles are identical either way" is only true while walking
straight.** Verified: `build_gait` commands FL ≡ FR and RL ≡ RR joint-for-joint
at `yaw_deg == 0`, so `reverse` does the entire left/right mirror. But a **turn**
commands left ≠ right — a body spin rotates every foot the *same* angular
direction about the centre, which is a rotation between the two sides, not a
reflection, so `reverse` alone cannot produce it. `body_twist_xy_path` converts
the arc into each leg's own local frame with `LEG_SIGN_X`, and `reverse` then
maps that to physical motion. The turn *kinematics* are verified in
`quadruped_sim --report` (body spins the correct direction, margin/rate hold),
but the sim does not model `reverse`, so **whether the `reverse` flags compose
correctly with a turn is a bench check** — do it on a stand, off the ground,
before any powered turning.

### Front/rear mirroring is handled in software — verified
The IK solves a **knee-forward** leg: at neutral stance the knee node sits at
local y = +18.8 cm. That matches the **rear** pair. The **front** pair is bolted
on turned round so its knees point back (toward the rear knees, as on a real
dog), so the front legs' stride is mirrored in software: `LEG_SIGN_Y = -1` for
the front pair, applied inside `body_twist_xy_path` (this replaced the old
`mirror_y=(leg_id in FRONT_LEGS)` flag; the two are equivalent and
`body_twist_xy_path` at `yaw == 0` reproduces the old gait term for term).

Verified numerically: during stance the front feet travel one way in their
*local* frame and the rear feet the opposite way, but `LEG_SIGN_Y` makes all
four move **backward in the body frame** together — which is what drives the
body forward. Without the flip the front legs would push against the rear.

It is a spatial mirror, **not** a time reversal. Reversing the cycle would also
shift each flipped leg half a cycle and scramble the FL → RR → FR → RL crawl
order.

**One thing `LEG_SIGN_Y` does NOT do, and must not be confused with:** it mirrors
the *stride direction*, not the *knee bend direction*. Whether a given knee angle
bends the joint fore or aft is a property of the assembly and the `reverse` flag
on that joint's `JointController`, not of the gait. So "the front knees face the
rear knees" is handled in two separate places — stride by `LEG_SIGN_Y` (verified
above, in software), bend direction by the per-joint `reverse` flags (still all
`False`, still a bench check — see the entry above this one). Getting
`LEG_SIGN_Y` right does not tell you the `reverse` flags are right.

A stale docstring on `GaitPath.update_params` used to say `mirror_y` applied to
"the rear pair, whose knees point forward", contradicting `FRONT_LEGS` three
screens above it. The code was always right (front pair mirrored); the comment
has been corrected.

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

### RESOLVED — `current_state` conflated operating mode with transient activity

`RobotState` was one field used for two unrelated things: MANUAL/AUTONOMOUS
(what the operator asked for) and RECOVERY (what the robot is doing right now).
Three bugs followed, the middle one able to drop the robot:

1. After any recovery the worker hard-set `current_state = MANUAL`, so a stumble
   silently dropped the robot out of GPS nav while the dashboard still read
   NAV: ON.
2. `nav_mode_callback` overwrote `current_state` unconditionally from a ROS
   executor thread. A NAV toggle landing between `handle_recovery()` staging a
   recovery and `_gait_serial_worker` reading it flipped the worker's
   `== RECOVERY` test false — **the recovery was never transmitted, and the
   aborted leg stayed frozen at zero PWM with `has_aborted` latched while the
   other three kept walking.** That is a fall.
3. The worker's state check raced that same callback.

**Fixed** by splitting into two fields with one writer each:
`current_mode` (`RobotMode.MANUAL` / `AUTONOMOUS`) written only by
`nav_mode_callback`, and `current_activity` (`RobotActivity.NORMAL` /
`RECOVERY`) written only by the recovery path. The worker gates on
`current_activity`, which the nav toggle cannot touch, so (2) is structurally
impossible now; clearing a recovery no longer resets the mode, so (1) is gone;
and there is no longer a single field two threads fight over, so (3) is gone.
`RobotState` remains as a back-compat alias of `RobotMode` — `.RECOVERY` was
deliberately removed from it so a stale reference raises instead of silently
comparing a mode against an activity. Verified against the source: no
`current_state` in executable code, nav callback touches only `current_mode`,
recovery touches only `current_activity`.

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

### RESOLVED — steering is now a body twist (yaw), not differential stride

Was: `chosen_direction` mapped to a left/right stride-length difference, which
turned the robot like a skid-steer — every foot moving straight fore/aft, the
two sides covering different ground, the feet scrubbing sideways as the body
yawed. The hip-roll joint did almost nothing during a turn. `GaitPath`'s
`direction_angle` — nominally the omnidirectional mechanism — was hardcoded to
`0` everywhere and, tested, only strafed the body (translated it diagonally)
without ever yawing it.

Now: `ik_and_gait.body_twist_xy_path()` takes a forward stride **and** a yaw per
cycle. Each planted foot arcs about the body centre — its neutral (hip) position
rotated `± yaw/2` across the stroke — so the four feet phased together drive the
body through a real rotation, with the hip-roll joint carrying the lateral part
of each arc (this is how CHAMP / MIT-Cheetah-lineage controllers turn).
`pi5_main`'s loop maps `chosen_direction` (+ = turn right) to a yaw rate plus a
forward stride that tapers to zero by `YAW_FULL_SPIN_DEG` (90°), so a hard
LEFT/RIGHT spins in place and a moderate heading error arcs while still
advancing.

Verified in `quadruped_sim.py --report` across the full command range:
`yaw_deg == 0` reproduces the old straight gait **term for term** (margin
+2.38 cm, 266 deg/s, 55.8 cm/4 cyc — unchanged); every turn keeps one leg
airborne, the FL→RR→FR→RL crawl order, joint rate ≤ 283 deg/s (79%), and all
foot targets reachable. Full spin is ~13 deg/cycle → ~16 deg/s → a 90° turn in
~5.5 s.

**Still open / notes:**
- *There is still no zero-velocity "keep standing, hold heading" travel state.*
  `chosen_direction = 0` means walk forward. STOP de-powers; a large obstacle,
  or a completed mission, makes the loop march in place (stride 0) — feet lift,
  body holds, no ground covered. That covers mission end (now RESOLVED, below),
  but it is not a true "stand still and hold this heading" state.
- *A "spin in place" drifts.* The phased crawl means only 3 feet are planted at
  any instant, so each single-leg lift during a spin leaves a few mm of
  un-cancelled translation — measured 3–8 cm of drift over a 53° spin, and
  slightly asymmetric left vs right (the front-leg mirror). Nav recomputes the
  bearing from GPS every cycle so it self-corrects; a true zero-drift spin would
  need the arc centred on the support-triangle centroid, not the body centre.
- *`direction_angle` in `GaitPath` is now dead* for production (still used by
  `gait_testing3.py`). Left in place; `build_gait` no longer calls `GaitPath`
  for the gait itself, only `swing_steps()` uses it (for the lift schedule).
- The `TURN_BODY_SHIFT_CM` ramp (2 → 4 cm of tripod lean as the turn tightens)
  is what keeps the margin up during a turn — see its comment. Straight walking
  is untouched.
- `chosen_direction` still means "travel direction" in MANUAL and "heading
  error" in AUTONOMOUS — one variable, two meanings. Harmless with the yaw
  mapping (both want "yaw toward this"), but the `current_state` conflation
  above is the related cleanup.

### RESOLVED — mission end now holds position; route is editable from the dashboard

`Navigator.calculate_nav` returns `None` once the waypoints are exhausted. The
control loop used to let `chosen_direction` fall through to the last manual
value, so the robot walked off forever. It now sets `chosen_direction = 0` and
`stride_scale = 0.0` on that `None` — the march-in-place command from the
avoidance work — so the robot picks its feet up on the spot, stays standing and
IMU-levelled, and waits for STAND or STOP. The avoidance planner's stride is
folded in with `min()` now, not assignment, so a CLEAR decision (1.0) can no
longer undo that hold.

This also depended on there being a route at all. There wasn't a way to enter
one — `MISSION_WAYPOINTS` was two hardcoded pairs in `main()`. Now:

- `Navigator` takes an optional list, defaults to empty, and is fully
  lock-guarded (`set_waypoints` / `get_waypoints` / `progress` /
  `mission_complete`) because the control loop reads it at ~100 Hz while a ROS
  executor thread may be rewriting it. `set_waypoints` validates lat/lon range
  and always restarts the route at index 0.
- New topic `/apex/navigation/waypoints` (`Float32MultiArray`, flat
  `[lat, lon, ...]`, empty = "no route"). `waypoints_callback` on the
  controller hands it to `Navigator`.
- `stream_server` gains `/waypoints` (GET, the browser's working copy) and
  `/set_waypoints` (POST JSON, validated, published). The status array grew two
  appended fields — `wp_index`, `wp_total` — behind a `len >= 17` guard, so an
  older dashboard still parses.
- The dashboard has a **Route** card: one latitude box and one longitude box per
  point, up/down to reorder, `×` to remove, **+ Add point**, and **Send route**
  (nothing reaches the robot until pressed). It shows "driving to waypoint N of
  M" / "route complete — holding position" / "no route loaded".

With no route loaded, flipping NAV MODE just marches in place rather than
driving toward stale coordinates. Verified end to end without hardware
(`Navigator` editing, arrival advance, concurrent-access smoke, `stream_server`
validation + publish, 17-field status parse, and the preview server's route
progression).

Note the drift caveat from the steering entry above still applies to any turn,
and there is still no separate "hold heading while stationary" state — but
mission end is no longer one of the things that needs it.

### RESOLVED — blocking sensor reads moved off the control loop
The `while rclpy.ok()` loop in `pi5_main.main()` used to call `imu.update()`,
`compass.get_heading()` and (once a second) the INA219 reads **inline, every
iteration**. Those are synchronous I2C transactions — a few ms on a healthy bus,
but on a NAKing or wedged bus the kernel blocks for its I2C timeout, and while it
blocked so did steering resends, the IMU reflex and the avoidance stride command.

`sensor_hub.SensorHub` now runs one poller thread per sensor (IMU 50 Hz, GPS +
compass 10 Hz, INA219 1 Hz); each thread owns its hardware object exclusively and
publishes into a lock-protected snapshot. The loop reads `imu_snapshot()` /
`nav_snapshot()` / `power_snapshot()` — dict copies, never hardware — each with a
staleness cutoff, so a dead or hung sensor degrades to the safe default (flat
attitude, hold last heading, skip the battery check) instead of stalling the
gait. Two failure signals: snapshot freshness (poll ran, reading was bad/old) and
`thread_alive()` (poller blocked inside a hardware call right now); the loop logs
which poller is stalled every ~5 s via `SensorHub.health()`.

`GPSReader.update()` was already non-blocking (drains the UART buffer only) — it
rides the nav poller just to keep all sensor I/O in one place.
`CompassReader.get_heading()` now returns `None` (not `0.0`) on an I2C failure so
a wedged bus is distinguishable from a genuine due-north reading; the hub holds
the last good heading across a `None`. Self-test: `python sensor_hub.py` →
`SENSOR HUB OK` (covers a failing sensor and a hung poller).

Still inline and still blocking, but sub-millisecond so not worth threading:
`controller.read_pico_lines()` (non-blocking by construction, same buffered-drain
pattern as GPS) and `audio_engine.play()` (already fire-and-forget — it spawns
its own daemon thread, `play()` itself just does an `os.path.exists` and a thread
start).

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

### Low-voltage alarm threshold may be unreachable — check which rail the INA219 is on

`pi5_main.LOW_VOLT_THRESHOLD = 4.75` V, compared against `INA219.get_voltage()`.
That is a sensible undervoltage trip for a **5 V regulated rail**, but the BOM
and README describe the INA219 as *battery* telemetry, and the electronics pack
is a **2S LiHV** — roughly 8.7 V full to 6.0 V empty. If it is wired across that
battery, 4.75 V is below fully-flat and `low_battery.wav` can never play; the
alarm is dead code. Decide which it is and set the threshold to match (~6.4 V
for a 2S pack, or leave 4.75 if it really is on the 5 V rail).

The current side of the same alarm is already bounded correctly: the ±320 mV
shunt range across 0.1 Ω saturates at 3.2 A and `MAX_CURRENT_MA` is 3000, so
that half can fire. Both are on the *electronics* supply either way — nothing
monitors the 3S motor pack, which is the one that actually gets hammered.

Verified correct while checking this, so it does not need re-deriving: config
word `0x399F` decodes to 32 V range / ±320 mV / 12-bit / continuous, calibration
2048 gives a 0.2 mA current LSB matching `raw * 0.2`, and the power LSB is 20×
that, matching `raw * 4.0` mW.

### RESOLVED — an out-of-range target left the motor at its last duty

`JointController.move_to()` returned early — before touching the PWM registers —
when `target_angle` was non-finite or outside ±360°. The guard itself is right
(it is what stops NaN surviving the clamp as full duty), but the bare `return`
meant the joint kept driving at whatever duty the previous call set. On a
saturated PID that is full duty into a stop, held until a good target arrives.

It also left `self.last_time` untouched, so the first good call after a run of
bad ones saw a `dt` inflated by the whole gap and dumped a large `error * dt`
straight into the integrator.

**Fixed:** the guard now zeroes both PWMs (coast), clears `integral` and
`prev_error` so nothing carries across the gap, and advances `last_time`. Still
low-likelihood — `pico_main` range-checks every payload before it reaches the
buffer — but it is now a safe failure rather than a latched one.

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
foot, against **4.76 cm** of commanded swing clearance (not the nominal 5.0 —
see "Swing clearance is 4.76 cm" above) — so the margin is ~0.76 cm.
**Verify the foot actually clears the ground on the bench.** Easing the velocity through those corners would help more
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

## Structural notes from the full review (no defect, worth knowing)

Verified correct during the review, recorded so nobody re-derives them:

- **Quadrature decode table is right.** `_QUAD_TABLE` covers exactly the eight
  valid single-step transitions with the correct signs; invalid/skipped
  transitions map to 0 rather than corrupting the count. `ppr = 28` is already
  the 4×-decoded count per motor revolution, which is what the both-edges
  both-channels ISR produces — so 28 × 99.5 / 360 = 7.74 ticks/deg is consistent
  and the resolution really is 0.129°.
- **Quaternion axis order is right.** Adafruit's `bno.quaternion` returns
  `(i, j, k, real)` and `_quat_to_pitch_roll(i, j, k, real)` takes them in that
  order.
- **The `GaitIK` roll-discontinuity guard never fires on a real gait.** Checked
  every step of all four legs with body shift on: 0 of 80 steps differed from
  the raw IK solution, so it is not silently freezing roll anywhere.
- **`GaitPath` is shared mutable state, used safely — by ordering, not design.**
  `build_gait()` calls `swing_steps()` (which mutates `self.path_gen`) and reads
  the result out *before* the per-leg loop mutates it again. Correct today,
  fragile if anyone reorders those lines. Same object is also reached from
  `single_leg_test`, but that is a separate process.
- **Deactivated legs still receive recovery and abort cleanup.** `handle_recovery`
  and the aborted-frame close both use `active_legs()` / `ser_list` rather than
  filtering deactivated ones. That is the safe direction — a debug flag should
  not suppress a fault response — but it is inconsistent with the normal gait
  path, which does skip them.
- **`engage_walking()` sleeps ~1.8 s inside a ROS callback.** It is on an
  executor thread and `MultiThreadedExecutor` has others, so nothing deadlocks,
  but that thread is blocked for the duration of the ramp.

From the second review pass (2026-08-29), all re-verified by running:

- **Nothing winds up at power-on.** `JointController(initial_angle=0)` and
  `current_targets = [0.0, 0.0, 0.0]` mean the first `move_to()` sees error = 0,
  which is inside the 1° deadband, so `output = 0` and `integral = 0`. Both PWMs
  stay at zero. The legs hold under nothing at all until the first Home press,
  which is the intent. `zero_at()` then clears `integral`/`prev_error`, so homing
  cannot carry a stale term into the first real move either.
- **Body-shift phase alignment is exact.** `apply_body_shift` samples the profile
  at `(j - offset_L) % N` and `_gait_serial_worker` writes buffer index
  `(i + offset_L) % N`, so at Pico tick `i` every leg uses profile index `i`. The
  two rotations cancel. Checked for all four legs at all 20 ticks.
- **March-in-place is genuinely a march.** At `stride_scale = 0` (BLOCKED),
  `body_twist_xy_path(fwd_cm=0, yaw_rad=0)` gives exactly zero horizontal foot
  travel — `max(|x|, |y|) = 0.0` — while the swing lift still peaks at 4.76 cm.
  The robot picks its feet up in place and stays IMU-levelled, which is the
  documented difference from STOP.
- **The avoidance planner cannot make the robot fall.** Its only outputs are a
  steering angle and a stride multiplier, both of which feed the same
  `build_gait()` path as manual steering; the stability margin and joint-rate
  checks in `quadruped_sim.py --report` cover the whole commanded range. A dead
  camera fails *open* (`reset()` + `AvoidState.OFF`, stride 1.0), so it degrades
  to ordinary walking rather than halting in a field.
- **A missing depth model degrades cleanly.** `ObstacleAvoider` constructs with
  `depth = None` rather than raising, `available` is False, `start()` no-ops,
  `plan()` returns the desired heading unchanged at stride 1.0, and the dashboard
  shows `AVOIDANCE: NO MODEL` with the toggle disabled.
- **`Decision`'s docstring said "saturating at +/-45"** while `MAX_STEER_DEG` is
  90. Comment only — corrected.

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
