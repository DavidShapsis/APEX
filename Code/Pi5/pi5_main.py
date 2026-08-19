import threading
import math
import time
import serial
import struct
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32, Int32MultiArray, Bool
import os

WAV_DIR = os.path.dirname(os.path.abspath(__file__))

# Core hardware and engine imports
from power_monitor import INA219
from inverse_kinematics.ik_and_gait import (
    InverseKinematics, GaitPath, GaitIK, RecoveryPath,
    LEG_FL, LEG_FR, LEG_RR, LEG_RL, LEG_ORDER, LEG_NAMES,
    LEFT_LEGS, FRONT_LEGS, phase_offsets, HOME_POSE, cartesian_ramp,
    body_shift_profile, apply_body_shift, attitude_height_offsets,
)
from audio import QuadrupedAudio
from webcam import USBWebcam
from stream_server import RobodogStreamer
from navigation import GPSReader, CompassReader, Navigator
from imu import IMU

class RobotState:
    MANUAL = 0
    AUTONOMOUS = 1
    RECOVERY = 2

# Leg identity, body geometry and mounting all live in ik_and_gait.py so that
# pi5_main and quadruped_sim cannot drift apart. build_gait() lays the second
# axis out in LEG_ORDER and _gait_serial_worker indexes it by leg id, which only
# lines up while LEG_ORDER is sorted.
assert LEG_ORDER == tuple(range(len(LEG_ORDER))), "LEG_ORDER must stay in leg-id order"

# Portion of the cycle a foot spends airborne. 0.25 keeps three feet planted at
# all times, which is what the [0, N/2, 3N/4, N/4] crawl sequencing needs.
SWING_FRACTION = 0.25
GAIT_STEPS = 20
STAND_HEIGHT = 36.0
SWING_HEIGHT = 5.0

# Must match Pico/pico_main.py STEP_TICK_MS -- used to time the startup ramp.
STEP_TICK_S = 0.040

# Zero, not 2.5. Three feet are planted at different points in the stance phase,
# so any downward push commands them to depths spanning ~2.2 cm -- on flat rigid
# ground they cannot all be there, and it becomes body bob or lost contact.
# Raise it only alongside closed-loop FSR contact for uneven terrain.
STANCE_PUSH = 0.0

# How far the body leans toward the supporting tripod before each lift. Without
# it the centre of mass rides the support triangle's edge with ~4 mm of margin;
# 2 cm takes that to ~2.2 cm. Costs foot workspace, so do not inflate it.
BODY_SHIFT_CM = 2.0

# IMU levelling. gain 1.0 would fully cancel the measured tilt in one update,
# which invites overshoot; 0.6 damps it. The limit caps how far one corner can
# be driven from the nominal stand height.
ATTITUDE_GAIN = 0.6
ATTITUDE_LIMIT_CM = 4.0

class PiQuadrupedController(Node):
    def __init__(self):
        super().__init__('pi5_main_node')
        
        # --- ROS 2 Publishers & Subscribers ---
        self.joint_pub = self.create_publisher(Float32MultiArray, '/apex/kinematics/joint_targets', 10)
        self.dir_sub = self.create_subscription(Int32, '/apex/navigation/cmd_dir', self.direction_callback, 10)
        self.nav_mode_sub = self.create_subscription(Bool, '/apex/navigation/nav_mode', self.nav_mode_callback, 10)

        # --- Homing / stand / go / stop / per-leg debug deactivation, driven
        # from the web dashboard ---
        self.home_leg_sub = self.create_subscription(Int32, '/apex/homing/cmd_home_leg', self.home_leg_callback, 10)
        self.stand_sub = self.create_subscription(Bool, '/apex/homing/cmd_stand', self.stand_callback, 10)
        self.go_sub = self.create_subscription(Bool, '/apex/homing/cmd_go', self.go_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/apex/homing/cmd_stop', self.stop_callback, 10)
        self.deactivate_leg_sub = self.create_subscription(
            Int32, '/apex/homing/cmd_deactivate_leg', self.deactivate_leg_callback, 10)
        self.reactivate_leg_sub = self.create_subscription(
            Int32, '/apex/homing/cmd_reactivate_leg', self.reactivate_leg_callback, 10)
        self.homing_status_pub = self.create_publisher(Int32MultiArray, '/apex/homing/status', 10)

        # --- Hardware Serial Setup ---
        self.pico_ports = ['/dev/ttyAMA0', '/dev/ttyAMA2', '/dev/ttyAMA3', '/dev/ttyAMA4']
        self.ser_list = []
        self.rx_buffers = []
        self.port_by_leg = {}   # leg id -> serial, filled from Pico announcements
        self.init_serial_ports()
        self.end_marker = b'\xFF' * 16          # gait: Pico loops the buffer
        self.oneshot_end_marker = b'\xFE' * 16  # recovery: Pico holds the final step
        self.home_marker = b'\xAB\xAB'          # zero one leg's encoder to HOME_POSE
        self.relax_marker = b'\xAC\xAC'         # cut holding torque, keep encoder tracking

        # --- Sub-Engine Initializations (Standardized to Centimeters) ---
        self.ik_engine = InverseKinematics({'a': 9.65, 'b': 26.84, 'c': 24.37})
        self.path_gen = GaitPath()
        self.recovery_engine = RecoveryPath(self.ik_engine)

        # Default straight-ahead gait, already in per-leg format so the rear pair
        # is mirrored from the very first frame. Nothing is sent to hardware yet --
        # see main(): the robot stays put until every leg is homed via the web UI.
        self.all_angles = self.build_gait(10.0, 10.0)

        # Static, feet-under-hips pose, identical for all four legs for the same
        # reason HOME_POSE is: the per-joint reverse flags handle the mirroring.
        stand_ik = self.ik_engine.calculate(0.0, 0.0, STAND_HEIGHT)
        self.stand_pose = (stand_ik.roll, stand_ik.pitch, stand_ik.knee)

        # --- Homing / stand / walk gating ---
        # No leg starts homed, so request_stand()/engage_walking() refuse to run
        # until the operator has homed every leg from the web UI. This is the
        # actual answer to "how does firmware know a leg is at HOME_POSE" --
        # it doesn't, the human says so by pressing the button.
        self.homed = {leg_id: False for leg_id in LEG_ORDER}
        self.standing = False
        self.walking_enabled = False

        # Debug/testing only: a deactivated leg is still fully computed every
        # gait tick (nothing about the loop changes), it just never receives
        # the resulting wire signal, so it holds whatever it was last told
        # while the other three keep walking normally.
        self.deactivated_legs = set()

        # System State Tracking Management
        self.current_state = RobotState.MANUAL
        self.last_sent_direction = 0
        self.last_sent_pitch = 0.0
        self.last_sent_roll = 0.0
        self.target_direction = 0
        self.filtered_heading = 0.0

        # --- Background Serial Worker Thread Management ---
        self.serial_lock = threading.Lock()
        self.gait_update_queue = None
        self.emergency_queue = None
        # FIFO of (marker_bytes, [(leg_id, serial), ...]) jobs -- Home (one leg)
        # and Stop (all legs) both go through this, so every wire write still
        # comes from the single worker thread.
        self.command_queue = []
        self.is_running = True
        
        self.gait_worker_thread = threading.Thread(target=self._gait_serial_worker, daemon=True)
        self.gait_worker_thread.start()

    def init_serial_ports(self):
        """Initializes connection to all 4 leg Picos."""
        for port in self.pico_ports:
            try:
                s = serial.Serial(port, baudrate=115200, timeout=0.1)
                self.ser_list.append(s)
                self.rx_buffers.append(b'')
                self.get_logger().info(f"UART setup successful: {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to open {port}: {e}")

    def read_pico_lines(self):
        """Drains every Pico UART without blocking, returning [(serial, line), ...].

        readline() is unusable here: in_waiting > 0 does not mean a full line is
        present, so a partial one blocks for the whole 0.1s timeout -- up to 0.4s
        across four ports, inside a loop that is meant to run at 100Hz.
        """
        messages = []
        for idx, s in enumerate(self.ser_list):
            try:
                pending = s.in_waiting
                if pending:
                    self.rx_buffers[idx] += s.read(pending)
                while b'\n' in self.rx_buffers[idx]:
                    raw, self.rx_buffers[idx] = self.rx_buffers[idx].split(b'\n', 1)
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if line:
                        messages.append((s, line))
            except Exception as e:
                self.get_logger().error(f"Serial read error on {s.port}: {e}")
        return messages

    def build_gait(self, left_stride, right_stride, roll_tilt=0.0, pitch_tilt=0.0,
                   height_z=STAND_HEIGHT):
        """Builds a [step][leg_id][roll, pitch, knee, is_swing] matrix.

        Stride length differs left/right to steer, and the front pair mirrors
        because it is mounted knees-back against the IK's knee-forward solution.
        On top of that:

          * body sway, phased with the lift sequence, so the centre of mass stays
            inside the support triangle instead of riding its edge;
          * differential leg height from the IMU, which actually levels the body
            (a common offset on all four legs only translates it).

        The second axis is ordered by LEG_ORDER, which is how
        _gait_serial_worker indexes it.
        """
        offsets = phase_offsets(GAIT_STEPS)
        swing_steps = self.swing_steps(height_z)
        shift = body_shift_profile(swing_steps, offsets, GAIT_STEPS, BODY_SHIFT_CM)
        dz = attitude_height_offsets(roll_tilt, pitch_tilt,
                                     gain=ATTITUDE_GAIN, limit_cm=ATTITUDE_LIMIT_CM)

        per_leg = {}
        for leg_id in LEG_ORDER:
            stride = left_stride if leg_id in LEFT_LEGS else right_stride
            self.path_gen.update_params(
                center_stride_y=0.0, center_height_z=height_z + dz[leg_id],
                length=stride, height1=SWING_HEIGHT, height2=STANCE_PUSH,
                direction_angle=0, swing_fraction=SWING_FRACTION,
                mirror_y=(leg_id in FRONT_LEGS)
            )
            path = apply_body_shift(self.path_gen.gait_xy_path, leg_id,
                                    offsets, shift, GAIT_STEPS)
            per_leg[leg_id] = GaitIK(self.ik_engine, path).get_gait_ik()

        num_steps = len(per_leg[LEG_FL])
        return [[per_leg[leg_id][i] for leg_id in LEG_ORDER] for i in range(num_steps)]

    def build_ramp(self, start_pose, target_by_leg, steps=40):
        """One-shot Cartesian ramp from a single fixed joint pose into a per-leg
        target, e.g. HOME_POSE into the stand pose, or the stand pose into the
        gait's first step. target_by_leg entries may carry a trailing is_swing
        flag (gait frames do) -- cartesian_ramp only looks at the first three.
        """
        ramp = {
            leg_id: cartesian_ramp(self.ik_engine, start_pose,
                                   target_by_leg[leg_id], steps=steps)
            for leg_id in LEG_ORDER
        }
        num_steps = len(ramp[LEG_FL])
        return [[ramp[leg_id][i] for leg_id in LEG_ORDER] for i in range(num_steps)]

    def home_leg_callback(self, msg):
        self.request_home_leg(msg.data)

    def stand_callback(self, msg):
        if msg.data:
            self.request_stand()

    def go_callback(self, msg):
        if msg.data:
            self.engage_walking()

    def stop_callback(self, msg):
        if msg.data:
            self.request_stop()

    def deactivate_leg_callback(self, msg):
        self.request_deactivate_leg(msg.data)

    def reactivate_leg_callback(self, msg):
        self.request_reactivate_leg(msg.data)

    def request_home_leg(self, leg_id):
        """Queues the wire command that zeroes one leg's encoders to HOME_POSE
        where it currently, physically is. Fire-and-forget: the Pico's
        HOMED,<id> ack, picked up by read_pico_lines() in the main loop, is
        what actually flips self.homed[leg_id].
        """
        if leg_id not in LEG_ORDER:
            self.get_logger().error(f"Cannot home unknown leg id {leg_id}")
            return
        s = self.port_by_leg.get(leg_id)
        if s is None:
            self.get_logger().error(
                f"Cannot home {LEG_NAMES.get(leg_id, leg_id)}: leg not identified yet.")
            return
        with self.serial_lock:
            self.command_queue.append((self.home_marker, [(leg_id, s)]))

    def request_stop(self):
        """Ends the active walk and cuts holding torque on every identified
        leg -- no active power, but the Pico's encoder ISR keeps counting
        regardless, so re-homing is NOT required to resume.

        This does not lower the robot into any kind of resting pose first. If
        it is standing or walking under its own weight when this is sent, the
        legs will sag or the robot may collapse the instant power is cut --
        there is no sit-down sequence. Safe once it is off the ground or
        otherwise supported; use with that in mind while it's load-bearing.
        """
        with self.serial_lock:
            self.walking_enabled = False
            self.standing = False
            targets = self.active_legs()
            if targets:
                self.command_queue.append((self.relax_marker, targets))

    def request_deactivate_leg(self, leg_id):
        """Debug/testing only: the gait loop keeps computing this leg's path
        exactly as before, it just stops being written to the wire, so the
        Pico holds whatever it was last told while the other three keep
        walking. Does not touch self.homed -- deactivating and reactivating a
        leg never requires re-homing it.
        """
        if leg_id not in LEG_ORDER:
            self.get_logger().error(f"Cannot deactivate unknown leg id {leg_id}")
            return
        with self.serial_lock:
            self.deactivated_legs.add(leg_id)

    def request_reactivate_leg(self, leg_id):
        if leg_id not in LEG_ORDER:
            return
        with self.serial_lock:
            self.deactivated_legs.discard(leg_id)

    def request_stand(self):
        """One-shot ramp from HOME_POSE into the static stand pose. Refuses
        unless every leg has been homed -- otherwise this ramps from wherever
        the leg happens to be, which is exactly what homing exists to prevent.
        """
        if not all(self.homed.values()):
            self.get_logger().error("Refusing to stand: not every leg is homed yet.")
            return False
        target = {leg_id: self.stand_pose for leg_id in LEG_ORDER}
        ramp = self.build_ramp(HOME_POSE, target)
        self.send_entire_gait(ramp, cycle=False)
        self.standing = True
        self.walking_enabled = False
        return True

    def engage_walking(self):
        """Ramps from the stand pose into the gait's first frame, then starts
        the continuous cyclic walk. Refuses unless the robot is already
        standing (which itself requires every leg homed) -- Home and Stand
        exist so this never has to guess where the legs physically are.
        """
        if self.walking_enabled:
            return True
        if not (self.standing and all(self.homed.values())):
            self.get_logger().error("Refusing to walk: home every leg and Stand first.")
            return False

        ramp = self.build_ramp(self.stand_pose, self.all_angles[0])
        self.send_entire_gait(ramp, cycle=False)
        time.sleep((len(ramp) - 1) * STEP_TICK_S + 0.2)
        self.send_entire_gait(self.all_angles)

        self.walking_enabled = True
        return True

    def publish_homing_status(self):
        """Wire layout: [homed x4, standing, walking, deactivated x4]."""
        with self.serial_lock:
            homed = [int(self.homed[leg_id]) for leg_id in LEG_ORDER]
            standing = int(self.standing)
            walking = int(self.walking_enabled)
            deactivated = [int(leg_id in self.deactivated_legs) for leg_id in LEG_ORDER]
        msg = Int32MultiArray()
        msg.data = homed + [standing, walking] + deactivated
        self.homing_status_pub.publish(msg)

    def swing_steps(self, height_z):
        """Buffer indices where the foot is clear of the ground."""
        self.path_gen.update_params(
            center_stride_y=0.0, center_height_z=height_z, length=10.0,
            height1=SWING_HEIGHT, height2=STANCE_PUSH, direction_angle=0,
            swing_fraction=SWING_FRACTION)
        return [i for i, s in enumerate(self.path_gen.gait_xy_path) if s[3]]

    def register_leg_announcement(self, s, line):
        """Handles a 'LEG,<id>' line from a Pico. True if the line was consumed."""
        if not line.startswith("LEG,"):
            return False
        try:
            leg_id = int(line.split(',')[1])
        except (IndexError, ValueError):
            self.get_logger().warn(f"Malformed leg announcement on {s.port}: {line}")
            return True

        if leg_id not in LEG_NAMES:
            self.get_logger().error(f"Pico on {s.port} announced invalid LEG_ID {leg_id}")
            return True

        claimed = self.port_by_leg.get(leg_id)
        if claimed is None:
            self.port_by_leg[leg_id] = s
            self.get_logger().info(f"Identified {LEG_NAMES[leg_id]} on {s.port}")
        elif claimed is not s:
            self.get_logger().error(
                f"Two Picos both claim {LEG_NAMES[leg_id]}: {claimed.port} and {s.port}. "
                "Set a unique LEG_ID on each board before walking.")
        return True

    def identify_legs(self, timeout=5.0):
        """Waits at startup for each Pico to announce which corner it drives.

        Boards repeat their announcement every 500ms until they receive a gait
        frame, so this works whether the Pi or the Picos powered up first.
        """
        deadline = time.time() + timeout
        while time.time() < deadline and len(self.port_by_leg) < len(self.ser_list):
            for s, line in self.read_pico_lines():
                self.register_leg_announcement(s, line)
            time.sleep(0.02)

        missing = [LEG_NAMES[i] for i in LEG_ORDER if i not in self.port_by_leg]
        if missing:
            self.get_logger().error(
                f"No LEG_ID announcement from: {', '.join(missing)}. "
                "Those legs will not be driven -- check each Pico's LEG_ID constant.")
        else:
            self.get_logger().info("All four legs identified.")
        return not missing

    def active_legs(self):
        """[(leg_id, serial), ...] for every identified leg.

        Falls back to ser_list order only when nothing announced itself, so a bench
        setup running older firmware still moves -- but in that mode the corner
        mapping is a guess, which identify_legs has already logged as an error.
        """
        if self.port_by_leg:
            return sorted(self.port_by_leg.items())
        return list(enumerate(self.ser_list))

    def direction_callback(self, msg):
        """Callback to handle arriving steering targets from other ROS 2 nodes."""
        with self.serial_lock:
            self.target_direction = msg.data

    def nav_mode_callback(self, msg):
        """Changes the robot's primary operating state machine channel."""
        with self.serial_lock:
            if msg.data:
                self.current_state = RobotState.AUTONOMOUS
                self.get_logger().info("Robot State Transited to: AUTONOMOUS_NAV")
            else:
                self.current_state = RobotState.MANUAL
                self.get_logger().info("Robot State Transited to: MANUAL")

    def publish_joints(self, angles_matrix):
        """Flattens gait matrix and publishes to the ROS world for visualization/logging."""
        msg = Float32MultiArray()
        flat_angles = []
        
        if len(angles_matrix) == 0:
            return
            
        # Detect if it's an interleaved multi-leg matrix shape: [Steps][Leg_Idx][Angles]
        if isinstance(angles_matrix[0][0], (list, tuple)):
            for multi_leg_step in angles_matrix:
                for leg in multi_leg_step:
                    flat_angles.extend([leg[0], leg[1], leg[2]])
        else:
            # Fallback for standard single-stream leg gait layouts [Steps][Angles]
            for step in angles_matrix:
                flat_angles.extend([step[0], step[1], step[2]])
                
        msg.data = flat_angles
        self.joint_pub.publish(msg)

    def send_entire_gait(self, angles_list, cycle=True):
        """Hand off the path array safely to the background worker thread.

        cycle=True is the normal walking gait, phase-offset per leg and looped
        by the Pico. cycle=False is a one-shot point-to-point move (the startup
        ramp) -- every leg plays the same tick index with no phase rotation,
        since the four paths are independent trajectories, not a shared cycle
        sampled at different offsets.
        """
        self.publish_joints(angles_list)
        with self.serial_lock:
            self.gait_update_queue = (angles_list, cycle)

    def handle_recovery(self, abort_payload, trigger_serial):
        """Processes recovery calculations safely and hands off execution to the
        background thread.

        Policy: go to neutral, not freeze in place. The leg that aborted gets
        its own Cartesian recovery path (get_recovery_gait, targeting x=y=0,
        z=STAND_HEIGHT -- the same point as self.stand_pose). The other three
        get sent directly to self.stand_pose too, so the robot isn't left
        trying to walk on three legs while the fourth recovers.

        The other three legs have no live position feedback -- the Pi only
        learns a leg's actual angle when THAT leg aborts and reports it -- so
        this cannot be a smooth Cartesian ramp like the boot/Stand ramps are.
        It's a direct single-target PID move, bounded by the same PWM limits
        as any ordinary gait step; walking-gait poses stay close to stand_pose
        by construction, so the jump is small next to the original HOME_POSE
        one.
        """
        try:
            parts = abort_payload.split(',')
            curr_roll, curr_pitch, curr_knee = float(parts[1]), float(parts[2]), float(parts[3])

            # Calculate current Cartesian coordinates via Forward Kinematics
            start_x, start_y, start_z = self.ik_engine.calculate_fk(curr_roll, curr_pitch, curr_knee)

            # Generate path back to neutral home stance
            recovery_gait = self.recovery_engine.get_recovery_gait(start_x, start_y, start_z)

            with self.serial_lock:
                other_targets = [(leg_id, s) for leg_id, s in self.active_legs()
                                  if s is not trigger_serial]
            neutral_hold = [list(self.stand_pose) + [0.0]] * 2

            # Stage details to background worker thread atomically
            with self.serial_lock:
                self.current_state = RobotState.RECOVERY
                self.emergency_queue = (trigger_serial, recovery_gait, other_targets, neutral_hold)
                self.gait_update_queue = None  # Clear outstanding standard gait steps
                self.standing = False
                self.walking_enabled = False

        except Exception as e:
            self.get_logger().error(f"Error in recovery parsing: {e}")

    def _gait_serial_worker(self):
        local_gait = None
        cycle_this_gait = True

        while self.is_running:
            recovery_job = None
            command_job = None

            with self.serial_lock:
                state_check = self.current_state

                if state_check == RobotState.RECOVERY and self.emergency_queue is not None:
                    recovery_job = self.emergency_queue
                    self.emergency_queue = None

                elif self.command_queue:
                    command_job = self.command_queue.pop(0)

                elif self.gait_update_queue is not None:
                    local_gait, cycle_this_gait = self.gait_update_queue
                    self.gait_update_queue = None

            # --- Lock is now RELEASED ---

            if command_job is not None:
                marker, cmd_targets = command_job
                for leg_id, s in cmd_targets:
                    try:
                        s.reset_output_buffer()
                        s.write(marker)
                    except Exception as e:
                        print(f"Command failed on {LEG_NAMES.get(leg_id, leg_id)}: {e}")
                continue

            if recovery_job is not None:
                trigger_serial, recovery_gait, other_targets, neutral_hold = recovery_job

                # The other three go to neutral first -- one 2-step one-shot
                # frame each, no phase offsets (independent point moves, same
                # reasoning as the startup/stand ramps).
                for leg_id, s in other_targets:
                    try:
                        s.reset_output_buffer()
                        s.write(b'\xAA\xAA')
                        for step in neutral_hold:
                            packed_data = struct.pack('ffff', float(step[0]), float(step[1]), float(step[2]), float(step[3]))
                            s.write(packed_data)
                        s.write(self.oneshot_end_marker)
                    except Exception as e:
                        print(f"Neutral recovery move failed on {LEG_NAMES.get(leg_id, leg_id)}: {e}")

                try:
                    trigger_serial.reset_output_buffer()
                    trigger_serial.write(b'\xAA\xAA')
                    for step in recovery_gait:
                        packed_data = struct.pack('ffff', float(step[0]), float(step[1]), float(step[2]), float(step[3]))
                        trigger_serial.write(packed_data)
                        time.sleep(0.01)
                    trigger_serial.write(self.oneshot_end_marker)
                except Exception as e:
                    print(f"Serial transmission crash during recovery: {e}")

                with self.serial_lock:
                    self.current_state = RobotState.MANUAL
                    # All four legs are now at stand_pose -- Go re-arms the walk
                    # with a fresh ramp, same as any other post-Stand start.
                    self.standing = True
                local_gait = None
                continue

            if local_gait is None:
                time.sleep(0.005)
                continue
                
            num_steps = len(local_gait)
            # A one-shot ramp is four independent point-to-point paths, not a
            # shared cycle sampled at different phases, so every leg plays the
            # same tick with no rotation.
            offsets = phase_offsets(num_steps) if cycle_this_gait else [0, 0, 0, 0]

            # Address legs by identity rather than by position in ser_list, so UART
            # wiring order is irrelevant and one failed port cannot shift every
            # leg's gait onto the wrong Pico.
            targets = self.active_legs()
            if not targets:
                local_gait = None
                continue

            with self.serial_lock:
                deactivated = set(self.deactivated_legs)

            # A deactivated leg gets no wire traffic at all this frame -- not
            # even the open/close markers -- so its Pico is left completely
            # alone holding whatever it last had, rather than being sent an
            # empty frame that would reset its buffer. Debug/testing only; the
            # per-tick loop below still runs and computes every leg's step
            # exactly as if nothing were deactivated.
            for leg_id, s in targets:
                if leg_id in deactivated:
                    continue
                try:
                    s.reset_output_buffer()
                    s.write(b'\xAA\xAA')
                except Exception:
                    pass

            aborted = False
            is_multi_leg_format = isinstance(local_gait[0][0], (list, tuple))

            # --- PROTECTED HYBRID SERIAL ENGINE LOOP ---
            for i in range(num_steps):
                with self.serial_lock:
                    if self.current_state == RobotState.RECOVERY:
                        aborted = True
                        break

                for leg_id, s in targets:
                    step_idx = (i + offsets[leg_id]) % num_steps

                    if is_multi_leg_format:
                        # Extract the path mapped for this specific leg profile
                        step = local_gait[step_idx][leg_id]
                    else:
                        # Fallback parsing for legacy standard single-stream arrays
                        step = local_gait[step_idx]

                    packed_data = struct.pack('ffff', float(step[0]), float(step[1]), float(step[2]), float(step[3]))
                    if leg_id in deactivated:
                        continue
                    try:
                        s.write(packed_data)
                    except Exception as e:
                        print(f"Serial write error on {LEG_NAMES.get(leg_id, leg_id)}: {e}")

                time.sleep(0.001)

            if aborted:
                # Close the frame on every port. A leg left mid-frame stays in
                # is_receiving, never steps, and cannot resync on the next frame.
                # Sent unconditionally, even to deactivated legs -- this is a
                # safety cleanup, not a normal move signal.
                for s in self.ser_list:
                    try:
                        s.write(self.end_marker)
                    except Exception:
                        pass
                local_gait = None
                cycle_this_gait = True
                continue

            marker = self.end_marker if cycle_this_gait else self.oneshot_end_marker
            for leg_id, s in targets:
                if leg_id in deactivated:
                    continue
                try:
                    s.write(marker)
                except Exception:
                    pass

            # The Pico cycles the buffer on its own, so one clean frame is
            # enough. Re-broadcasting would flush this frame mid-flight on the
            # next pass (~22ms loop vs ~29.3ms wire time) and it would never
            # arrive complete.
            local_gait = None
            cycle_this_gait = True
                
    def close_hardware(self):
        """Gracefully closes all hardware serial lines."""
        self.is_running = False
        if hasattr(self, 'gait_worker_thread'):
            self.gait_worker_thread.join(timeout=0.2)
        for s in self.ser_list:
            if s.is_open:
                s.close()

def main():
    rclpy.init(args=None)

    # IMU Configuration
    imu = IMU(sda_pin="D0", scl_pin="D1", bus_id=13, window_size=12)
    print("IMU setup successful")

    # Navigation Configuration
    MISSION_WAYPOINTS = [(41.056, -74.145), (41.057, -74.146)] 
    gps = GPSReader(uart_path='/dev/ttyUSB0', baudrate=9600)
    compass = CompassReader(sda_pin=2, scl_pin=3)
    nav_engine = Navigator(MISSION_WAYPOINTS)

    # Vision
    cam = USBWebcam(device_index="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_Camera_SN0001-video-index0")
    streamer = RobodogStreamer()

    def camera_loop():
        while rclpy.ok():
            frame = cam.get_frame()
            if frame is not None:
                streamer.update_frame(frame)
            time.sleep(0.03)

    streamer.run()
    threading.Thread(target=camera_loop, daemon=True).start()
    print("Vision and Stream components online")

    # Telemetry & Audio System
    power_monitor = INA219(bus_id=3)
    audio_engine = QuadrupedAudio("30:8D:EB:5D:AC:11")
    LOW_VOLT_THRESHOLD = 4.75
    # The +/-320mV shunt range across 0.1 ohm saturates at 3.2A, so anything
    # above that can never be reached and the alarm would never fire.
    MAX_CURRENT_MA = 3000.0
    AUDIO_COOLDOWN = 10.0
    last_power_check = time.time()
    last_audio_warning = 0
    last_status_publish = time.time()

    controller = PiQuadrupedController()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(streamer)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("ROS 2 Unified Multi-Node Infrastructure Started")

    # Learn which UART drives which corner before anything is commanded to move.
    controller.identify_legs()

    # Nothing is sent to the legs at boot. Each Pico holds its current position
    # under light PID (target 0,0,0 -- wherever it powered on) until the
    # operator homes every leg, presses Stand, then Go from the web dashboard.
    # See PiQuadrupedController.request_home_leg / request_stand / engage_walking.
    print("Boot complete. Legs are NOT driven -- home each leg from the web "
          "dashboard, then Stand, then Go.")

    try:
        initial_head = compass.get_heading()
        controller.filtered_heading = initial_head
        print(f"Compass tracking initialized successfully at: {initial_head:.2f}°")
    except Exception as e:
        print(f"[Hardware Warning] Failed to fetch initial compass sync: {e}")
    
    try:
        while rclpy.ok():
            try:
                current_time = time.time()
                gps.update()
                
                raw_head = compass.get_heading()
                # Blend along the shortest arc. Averaging raw degrees sends the
                # estimate the long way round every time the robot crosses north
                # (filtered=359, raw=1 would give 305).
                head_delta = (raw_head - controller.filtered_heading + 180.0) % 360.0 - 180.0
                controller.filtered_heading = (controller.filtered_heading + 0.15 * head_delta) % 360.0

                with controller.serial_lock:
                    snap_state = controller.current_state
                    snap_target_dir = controller.target_direction
                    snap_last_dir = controller.last_sent_direction
                    snap_last_pitch = controller.last_sent_pitch
                    snap_last_roll = controller.last_sent_roll
                
                # update() reports failure by returning None rather than raising,
                # so this must be checked -- otherwise a dead IMU silently feeds
                # its last good attitude into the gait forever.
                if imu.update() is not None:
                    roll_tilt = imu.get_roll()
                    pitch_tilt = imu.get_pitch()
                else:
                    roll_tilt, pitch_tilt = 0.0, 0.0
                    if int(current_time) % 2 == 0:
                        print("[Hardware Error] IMU read failed; stabilization disabled")

                # Levelling is differential leg height, computed per leg inside
                # build_gait via attitude_height_offsets -- there is no common
                # X/Y correction any more, because that only translated the body.
                if abs(roll_tilt) > 8.0 or abs(pitch_tilt) > 8.0:
                    if int(current_time) % 2 == 0:
                        print(f"[IMU Warning] Large Tilt! Roll: {roll_tilt:.2f}, Pitch: {pitch_tilt:.2f}")

                chosen_direction = snap_target_dir
                if snap_state == RobotState.AUTONOMOUS:
                    if not gps.has_fix:
                        # Without a fix lat/lon are 0.0, which would produce a
                        # confident bearing from the Gulf of Guinea.
                        if int(current_time) % 2 == 0:
                            print("[NAV] No GPS fix; ignoring waypoint guidance")
                    else:
                        nav_data = nav_engine.calculate_nav(gps.lat, gps.lon, controller.filtered_heading)
                        if nav_data is not None:
                            chosen_direction = nav_data["turn"]

                dir_delta = abs(chosen_direction - snap_last_dir) > 5
                pitch_delta = abs(pitch_tilt - snap_last_pitch) > 1.5
                roll_delta = abs(roll_tilt - snap_last_roll) > 1.5

                # Gated on walking_enabled: home every leg, Stand, then Go from
                # the web dashboard arms this. Before that the legs simply hold
                # wherever homing/Stand left them.
                if controller.walking_enabled and (dir_delta or pitch_delta or roll_delta):
                    if int(current_time) % 2 == 0:
                        print(f"[IMU Reflex] Levelling. Roll {roll_tilt:+.2f} deg, "
                              f"pitch {pitch_tilt:+.2f} deg")

                    # --- TRUE BODY STEERING (DIFFERENTIAL) ARCHITECTURE ---
                    steering_factor = chosen_direction / 45.0
                    steering_factor = max(-1.0, min(1.0, steering_factor))

                    base_stride = 10.0

                    # Inside vs Outside stride length scaling calculations
                    left_side_stride = base_stride * (1.0 + (0.5 * steering_factor))
                    right_side_stride = base_stride * (1.0 - (0.5 * steering_factor))

                    new_angles = controller.build_gait(
                        left_side_stride, right_side_stride,
                        roll_tilt=roll_tilt, pitch_tilt=pitch_tilt
                    )
                    controller.send_entire_gait(new_angles)

                    with controller.serial_lock:
                        controller.last_sent_direction = chosen_direction
                        controller.last_sent_pitch = pitch_tilt
                        controller.last_sent_roll = roll_tilt

                if current_time - last_power_check > 1.0:
                    v = power_monitor.get_voltage()
                    c = power_monitor.get_current()
                    if (v < LOW_VOLT_THRESHOLD or c > MAX_CURRENT_MA) and (current_time - last_audio_warning > AUDIO_COOLDOWN):
                        audio_engine.play(os.path.join(WAV_DIR, "low_battery.wav"))
                        last_audio_warning = current_time
                    last_power_check = current_time

                for s, line in controller.read_pico_lines():
                    if controller.register_leg_announcement(s, line):
                        continue
                    if line.startswith("HOMED,"):
                        try:
                            homed_leg = int(line.split(',')[1])
                            with controller.serial_lock:
                                controller.homed[homed_leg] = True
                            print(f"{LEG_NAMES.get(homed_leg, homed_leg)} homed.")
                        except (IndexError, ValueError):
                            pass
                        continue
                    if line.startswith("ABORTED"):
                        print(f"Hardware Stall Warning on UART: {s.port}")
                        audio_engine.play(os.path.join(WAV_DIR, "abort_sound.wav"))
                        controller.handle_recovery(line, s)

                if current_time - last_status_publish > 0.2:
                    controller.publish_homing_status()
                    last_status_publish = current_time

                time.sleep(0.01)
            except Exception as loop_err:
                print(f"[RUNTIME WARNING] Iteration skipped due to error: {loop_err}")
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nShutting down controller hardware nodes safely...")
    finally:
        controller.close_hardware()
        cam.release()
        controller.destroy_node()
        streamer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()