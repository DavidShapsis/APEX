import time
import struct 
from machine import UART, Pin
from fsr import FSR
from motor_control import JointController

# ==============================================================================
# LEG IDENTITY -- SET THIS BEFORE FLASHING. EVERY BOARD NEEDS A DIFFERENT VALUE.
# ==============================================================================
#   0 = Front Left     1 = Front Right     2 = Rear Right     3 = Rear Left
#
# The Pi reads this over UART at startup to learn which port drives which corner,
# so the UART wiring order stops mattering. But two boards sharing a value, or one
# set to the wrong corner, produces a wrong gait phase and the robot falls over.
# The Pi logs the map it detects -- check it before any powered walking.
LEG_ID = 0
# ==============================================================================

print("Imports successful")

# --- Setup Joints ---
roll_j  = JointController(rpwm_pin=3, lpwm_pin=2, en_pin=6, enc_a_pin=4, enc_b_pin=5, 
                          gear_ratio=99.5, ppr=28, reverse=False, initial_angle=0)
pitch_j = JointController(rpwm_pin=11, lpwm_pin=10, en_pin=7, enc_a_pin=8, enc_b_pin=9, 
                          gear_ratio=99.5, ppr=28, reverse=False, initial_angle=0)
knee_j  = JointController(rpwm_pin=15, lpwm_pin=14, en_pin=22, enc_a_pin=12, enc_b_pin=13, 
                          gear_ratio=99.5, ppr=28, reverse=False, initial_angle=0)
print("Joint Controllers setup successful")

# Which pin carries THIS leg's own foot sensor.
#
# This board drives one leg, and the abort test is "MY foot touched down while I
# should be airborne". It has to read one sensor. It used to read all four
# (FSR(16..19)) and collapse them with any(), which is wrong by construction: the
# crawl gait always has three feet planted, so any() is True for the whole of
# every swing phase and the leg aborted on its first step, every cycle. It never
# bit only because nothing is wired to these pins yet.
#
# Set this to the pin this leg's foot sensor is actually on before wiring them.
FSR_PIN = 16
foot = FSR(FSR_PIN)
print("FSR setup successful")

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), rxbuf=1024)
print("UART setup successful")
print("--------------------")

# --- Protocol ---
START_MARKER = b'\xAA\xAA'
HOME_MARKER = b'\xAB\xAB'           # web UI "Home this leg" command, no payload
RELAX_MARKER = b'\xAC\xAC'          # web UI "Stop" command, no payload
CYCLE_END_MARKER = b'\xFF' * 16     # gait: loop the buffer indefinitely
ONESHOT_END_MARKER = b'\xFE' * 16   # recovery: hold on the final step
PAYLOAD_SIZE = 16
ANGLE_LIMIT = 360.0
# Desync guard, not a design limit: an overlong frame means the stream lost
# sync mid-payload, and this stops the buffer growing until MemoryError.
# It MUST stay above every frame the Pi legitimately sends, and the biggest of
# those is the Stand/Go Cartesian ramp -- pi5_main.build_ramp(steps=40) emits
# steps+1 = 41 entries. This was 32, which silently threw the Stand and Go
# ramps away: the leg never left its homed pose and the robot could not stand.
# pi5_main.PICO_MAX_FRAMES mirrors this number and refuses to send past it.
MAX_GAIT_STEPS = 48

# Must match ik_and_gait.HOME_POSE on the Pi. Zeroing here only redefines what
# "current position" means -- it does not move the leg -- so this has to be the
# exact pose the leg was physically placed in by hand before homing.
HOME_POSE = (90.0, 0.0, 180.0)

# How often the step clock is re-aligned to a frame boundary.
#
# Every leg's end marker lands within a millisecond or two of the others, so
# resetting last_step_time there is a perfect cross-leg sync -- but it also
# throws away the partially-elapsed tick, and the Pi re-sends far more often
# than every 40 ms. Doing it on EVERY frame meant the tick never expired at all:
# measured 0 gait steps in 20 s for any re-send faster than 69 ms (29.3 ms of
# wire time + the 40 ms tick). The robot stood still while the Pi believed it
# was walking.
#
# So sync on a timer instead. Between syncs each board free-runs on its own
# crystal; at a worst-case 100 ppm spread that is 0.5 ms of drift over 5 s,
# against the 40 ms tick and the one-tick guard band between each leg's swing
# window. Cost is at most one truncated tick per 5 s, i.e. under 1% of walking
# speed.
RESYNC_INTERVAL_MS = 5000

# --- State Management ---
gait_buffer = []
# Frames land HERE while a new one is arriving, and are swapped into
# gait_buffer only once its end marker is parsed. gait_buffer used to be
# cleared at START_MARKER and refilled in place, which meant step 3 had nothing
# to play for the whole ~29.3 ms a frame takes on the wire -- so every steering
# or IMU update froze the gait for the duration, on top of the step-clock reset
# above. Staging lets the leg keep walking the previous cycle until the new one
# is complete, which is also what makes a desynced or truncated frame harmless.
rx_buffer = []
is_receiving = False
has_aborted = False
cycle_buffer = True
current_step_index = 0
# Length of the cyclic buffer that was playing when the current frame started
# arriving, or 0 if a one-shot was playing. Used to decide whether an incoming
# frame is a re-send of the same walking cycle (keep the phase) or something
# new (start at 0) -- see the end-marker handling.
prev_cycle_len = 0
# Web UI "Stop": no active holding torque, but the encoder ISR keeps counting
# regardless of this flag, so position (and homing) survives being unpowered.
# Resumes automatically the moment a real command arrives -- see HOME_MARKER
# and the end-of-frame handling below -- no separate "resume" command exists.
powered = True
# 40ms, not 20. At 20ms the gait demands 469 deg/s at the knee through liftoff and
# touchdown, against 360 deg/s free speed on the 5302 at 99.5:1 -- the PID saturates
# and the foot lands late. 40ms keeps the peak at ~235 deg/s (65% of free speed,
# leaving room for load) and costs walking speed: 16.7 cm/s instead of 33.3.
# Lower it only after checking on the bench that the legs still track their targets.
STEP_TICK_MS = 40
last_step_time = time.ticks_ms()
last_resync = time.ticks_ms()
prev_byte = b''

# --- Leg Identity Announcement ---
# Repeats until the Pi sends a first complete frame, so the Pi still learns the
# map when it boots after the Picos rather than before them.
ANNOUNCE_INTERVAL_MS = 500
identified = False
last_announce = time.ticks_ms()

# Holds at 0,0,0 -- wherever the leg physically is at power-on -- until either
# a HOME command or a gait frame arrives. move_to() drives toward this every
# loop regardless, so an unhomed leg just sits under light PID hold, not slack.
current_targets = [0.0, 0.0, 0.0]
# True while the zero-PWM branch is holding drive off, so PID state can be reset
# once on the transition back rather than every pass.
drive_was_cut = False

while True:
    # 1. READ UART (Binary Protocol Parser)
    # Drains everything buffered each pass; a single read per loop falls behind
    # the 16-byte-per-1.4ms arrival rate and overruns the RX buffer.
    while True:
        if not is_receiving:
            # Scan a byte at a time. Reading in 2-byte pairs can never recover
            # from an odd-byte misalignment, because the marker and the payload
            # are both even-length so the pairing parity never shifts.
            if not uart.any():
                break
            byte = uart.read(1)
            candidate = prev_byte + byte
            if candidate == START_MARKER:
                # Remember what was playing so the end-marker handler can tell a
                # re-send of the walking cycle from a genuinely new trajectory.
                prev_cycle_len = len(gait_buffer) if cycle_buffer else 0
                rx_buffer = []
                is_receiving = True
                has_aborted = False
                # A new frame is the Pi's answer to whatever went wrong, so give
                # the joints another go. If the jam is still physical they will
                # simply re-latch STALL_TIMEOUT_S later -- a retry every 1.5 s,
                # not a tight loop at full duty.
                roll_j.clear_stall()
                pitch_j.clear_stall()
                knee_j.clear_stall()
                prev_byte = b''
            elif candidate == HOME_MARKER:
                # Manual homing: the leg was positioned by hand, so "current
                # position" IS HOME_POSE now -- redefine the encoder zero to
                # match without commanding any motion, then hold there.
                roll_j.zero_at(HOME_POSE[0])
                pitch_j.zero_at(HOME_POSE[1])
                knee_j.zero_at(HOME_POSE[2])
                current_targets = list(HOME_POSE)
                gait_buffer = []
                rx_buffer = []
                is_receiving = False
                has_aborted = False
                powered = True   # a Home command means "now hold here"
                uart.write("HOMED,%d\n" % LEG_ID)
                prev_byte = b''
            elif candidate == RELAX_MARKER:
                # Stop: cut active holding torque. Deliberately does not touch
                # gait_buffer, current_targets or the encoder -- only step 4
                # below reads `powered`, so position tracking is untouched and
                # nothing needs re-homing once power resumes.
                powered = False
                prev_byte = b''
            else:
                prev_byte = byte
        else:
            if uart.any() < PAYLOAD_SIZE:
                break
            full_payload = uart.read(PAYLOAD_SIZE)

            if full_payload == CYCLE_END_MARKER or full_payload == ONESHOT_END_MARKER:
                cycle_buffer = full_payload == CYCLE_END_MARKER
                is_receiving = False
                # An empty frame keeps whatever was already playing rather than
                # leaving the leg with nothing to step through.
                same_cycle = (cycle_buffer and rx_buffer
                              and prev_cycle_len == len(rx_buffer))
                if rx_buffer:
                    gait_buffer = rx_buffer
                rx_buffer = []

                # KEEP THE PHASE across a re-send of the same walking cycle.
                # The Pi rebuilds and re-sends the whole gait whenever steering
                # or the IMU tilt changes, which is often. Restarting at index 0
                # every time teleports the foot from wherever it was in the
                # stroke to the start of swing -- measured worst case 25.6 deg
                # at a joint / 17.6 cm at the foot, and kp saturates above
                # 1.25 deg, so that is a full-duty slam several times a second.
                # Continuing at the same index leaves only the geometry
                # difference: 7.1 deg / 4.2 cm for a hard turn, and exactly zero
                # for an IMU-only nudge.
                #
                # Same length + both cyclic is the test for "this is the same
                # 20-step walking cycle, re-rendered". Anything else -- a
                # one-shot ramp, or a cycle of a different length -- starts at 0,
                # because a ramp genuinely has to run from its first entry.
                if same_cycle:
                    current_step_index = current_step_index % len(gait_buffer)
                    # Step clock deliberately left alone -- see
                    # RESYNC_INTERVAL_MS. Re-aligning here on every frame is
                    # what used to freeze the gait outright.
                    if time.ticks_diff(time.ticks_ms(), last_resync) > RESYNC_INTERVAL_MS:
                        last_step_time = time.ticks_ms()
                        last_resync = last_step_time
                else:
                    # A genuinely new trajectory (ramp, recovery, a cycle of a
                    # different length) starts at its first entry, and that IS
                    # the moment to re-align all four boards.
                    current_step_index = 0
                    last_step_time = time.ticks_ms()
                    last_resync = last_step_time
                prev_byte = b''
                identified = True   # the Pi is talking to us; stop announcing
                powered = True      # a real frame arrived -- go drive it
            else:
                try:
                    parts = list(struct.unpack('ffff', full_payload))
                except Exception:
                    parts = None
                # Drops non-finite and out-of-range angles, which a desynced
                # stream produces and which would otherwise reach the motors.
                if parts is not None and all(-ANGLE_LIMIT <= v <= ANGLE_LIMIT for v in parts[:3]):
                    rx_buffer.append(parts)
                    if len(rx_buffer) > MAX_GAIT_STEPS:
                        # Overlong frame means the stream desynced mid-payload and
                        # the terminator is being straddled. Abandon it and rescan
                        # rather than growing the buffer until MemoryError. Only
                        # the staged copy is thrown away, so the leg carries on
                        # walking the last good cycle instead of freezing.
                        rx_buffer = []
                        is_receiving = False
                        prev_byte = b''

    # 1b. ANNOUNCE LEG IDENTITY until the Pi starts sending us gait frames
    if not identified and time.ticks_diff(time.ticks_ms(), last_announce) > ANNOUNCE_INTERVAL_MS:
        uart.write("LEG,%d\n" % LEG_ID)
        last_announce = time.ticks_ms()

    # 2a. STALL CHECK (The Abort Logic, hardware-damage branch)
    # A joint whose PID has been pinned at full duty without closing its error
    # has met something it cannot move. JointController latches .stalled and has
    # already zeroed that joint's PWM; raise the same ABORTED path the FSR uses
    # so the Pi brings the whole robot to a safe pose instead of leaving three
    # legs walking. Without this the joint simply held 100% duty until something
    # burned out -- see KNOWN_ISSUES.
    stalled_joint = None
    for name, j in (("roll", roll_j), ("pitch", pitch_j), ("knee", knee_j)):
        if j.stalled:
            stalled_joint = name
            break
    if stalled_joint is not None and not has_aborted:
        uart.write("STALL,%d,%s\n" % (LEG_ID, stalled_joint))
        msg = f"ABORTED,{roll_j.current_angle},{pitch_j.current_angle},{knee_j.current_angle}\n"
        uart.write(msg)
        has_aborted = True
        gait_buffer = []

    # 2b. GROUND CHECK (The Abort Logic)
    # This leg's own foot only -- see FSR_PIN.
    own_touchdown = foot.state

    # 3. CHOOSE TARGETS (every STEP_TICK_MS)
    # `is_receiving` is NOT a condition here any more: gait_buffer is the last
    # COMPLETE frame (an arriving one lands in rx_buffer), so the leg keeps
    # walking normally while the next frame is on the wire.
    if gait_buffer and not has_aborted:
        current_step_swing = gait_buffer[current_step_index][3] > 0.5
        if own_touchdown and current_step_swing:
            msg = f"ABORTED,{roll_j.current_angle},{pitch_j.current_angle},{knee_j.current_angle}\n"
            uart.write(msg)
            has_aborted = True
            gait_buffer = []
        else:
            elapsed = time.ticks_diff(time.ticks_ms(), last_step_time)
            if elapsed >= STEP_TICK_MS:
                # Advance by exactly one tick instead of resetting to now.
                # Resetting made the real cadence 40 ms PLUS however long the
                # pass took (~1-2 ms of loop, more while a frame is arriving),
                # so it tracked each board's own jitter -- tolerable while every
                # end marker re-synced the clock, but not now that a re-sync is
                # only every RESYNC_INTERVAL_MS. Fixed-step keeps the four legs
                # phase-locked to their crystals alone.
                if elapsed >= 2 * STEP_TICK_MS:
                    last_step_time = time.ticks_ms()      # fell behind; snap
                else:
                    last_step_time = time.ticks_add(last_step_time, STEP_TICK_MS)
                if cycle_buffer:
                    current_step_index = (current_step_index + 1) % len(gait_buffer)
                elif current_step_index < len(gait_buffer) - 1:
                    # One-shot trajectory (recovery): advance to the end, then hold.
                    current_step_index += 1
                current_targets = gait_buffer[current_step_index]

    # 4. EXECUTE CLOSED LOOP PID UPDATES
    # FIX: Run loop if we aren't aborted, regardless of incoming background updates.
    if has_aborted or not powered:
        # Same zero-PWM fallback for both cases: an abort is a fault latch that
        # needs recovery, "not powered" is the deliberate Stop command -- either
        # way the joint gets no drive current. The encoder ISR is independent of
        # this branch and keeps counting, so current_angle stays valid.
        roll_j.forward_pwm.duty_u16(0)
        roll_j.backward_pwm.duty_u16(0)
        pitch_j.forward_pwm.duty_u16(0)
        pitch_j.backward_pwm.duty_u16(0)
        knee_j.forward_pwm.duty_u16(0)
        knee_j.backward_pwm.duty_u16(0)
        drive_was_cut = True
    else:
        if drive_was_cut:
            # Drive has been off for an unknown length of time (a Stop can last
            # minutes). Without this the first move_to() sees a dt spanning the
            # whole gap and dumps error*dt into the integrator, saturating it in
            # one call. Same stale-dt failure the out-of-range guard already fixes.
            roll_j.reset_pid()
            pitch_j.reset_pid()
            knee_j.reset_pid()
            drive_was_cut = False
        roll_j.move_to(current_targets[0])
        pitch_j.move_to(current_targets[1])
        knee_j.move_to(current_targets[2])

    time.sleep_ms(1)