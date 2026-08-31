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

fsrs = [FSR(16), FSR(17), FSR(18), FSR(19)]
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

# --- State Management ---
gait_buffer = []
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
                gait_buffer = []
                is_receiving = True
                has_aborted = False
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
                if (cycle_buffer and gait_buffer
                        and prev_cycle_len == len(gait_buffer)):
                    current_step_index = current_step_index % len(gait_buffer)
                else:
                    current_step_index = 0

                # Reset regardless: all four boards receive their end marker
                # within a few ms of each other, so this re-aligns the 40 ms
                # tick across legs and stops them free-running apart.
                last_step_time = time.ticks_ms()
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
                    gait_buffer.append(parts)
                    if len(gait_buffer) > MAX_GAIT_STEPS:
                        # Overlong frame means the stream desynced mid-payload and
                        # the terminator is being straddled. Abandon it and rescan
                        # rather than growing the buffer until MemoryError.
                        gait_buffer = []
                        is_receiving = False
                        prev_byte = b''

    # 1b. ANNOUNCE LEG IDENTITY until the Pi starts sending us gait frames
    if not identified and time.ticks_diff(time.ticks_ms(), last_announce) > ANNOUNCE_INTERVAL_MS:
        uart.write("LEG,%d\n" % LEG_ID)
        last_announce = time.ticks_ms()

    # 2. GROUND CHECK (The Abort Logic)
    any_touchdown = any(f.state for f in fsrs)

    # 3. CHOOSE TARGETS (Every 20ms)
    # Target updates only advance through the buffer steps once receiving is complete
    if gait_buffer and not is_receiving and not has_aborted:
        current_step_swing = gait_buffer[current_step_index][3] > 0.5
        if any_touchdown and current_step_swing:
            msg = f"ABORTED,{roll_j.current_angle},{pitch_j.current_angle},{knee_j.current_angle}\n"
            uart.write(msg)
            has_aborted = True
            gait_buffer = []
        else:
            if time.ticks_diff(time.ticks_ms(), last_step_time) > STEP_TICK_MS:
                last_step_time = time.ticks_ms()
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
    else:
        roll_j.move_to(current_targets[0])
        pitch_j.move_to(current_targets[1])
        knee_j.move_to(current_targets[2])

    time.sleep_ms(1)