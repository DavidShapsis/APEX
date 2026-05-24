import threading
import math
import time
import serial
import struct
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32, Bool
import os

WAV_DIR = os.path.dirname(os.path.abspath(__file__))

# Core hardware and engine imports
from power_monitor import INA219
from InverseKinematics.ik_and_gait import InverseKinematics, GaitPath, GaitIK, RecoveryPath
from audio import QuadrupedAudio
from webcam import USBWebcam
from stream_server import RobodogStreamer
from navigation import GPSReader, CompassReader, Navigator
from imu import IMU

class RobotState:
    MANUAL = 0
    AUTONOMOUS = 1
    RECOVERY = 2

class PiQuadrupedController(Node):
    def __init__(self):
        super().__init__('pi5_main_node')
        
        # --- ROS 2 Publishers & Subscribers ---
        self.joint_pub = self.create_publisher(Float32MultiArray, '/apex/kinematics/joint_targets', 10)
        self.dir_sub = self.create_subscription(Int32, '/apex/navigation/cmd_dir', self.direction_callback, 10)
        self.nav_mode_sub = self.create_subscription(Bool, '/apex/navigation/nav_mode', self.nav_mode_callback, 10)
        self.nav_mode_pub = self.create_publisher(Bool, '/apex/navigation/nav_mode', 10)
        
        # --- Hardware Serial Setup ---
        self.pico_ports = ['/dev/ttyAMA0', '/dev/ttyAMA2', '/dev/ttyAMA3', '/dev/ttyAMA4']
        self.ser_list = []
        self.init_serial_ports()
        self.end_marker = b'\xFF' * 16

        # --- Sub-Engine Initializations (Standardized to Centimeters) ---
        self.ik_engine = InverseKinematics({'a': 9.65, 'b': 26.84, 'c': 24.37})
        self.path_gen = GaitPath()
        self.path_gen.update_params(center_stride_y=0.0, center_height_z=36.0, length=10.0, height1=5.0, height2=2.5, direction_angle=0)
        
        self.recovery_engine = RecoveryPath(self.ik_engine)
        self.gait_processor = GaitIK(self.ik_engine, self.path_gen.gait_xy_path)
        self.all_angles = self.gait_processor.get_gait_ik()
        
        # System State Tracking Management
        self.current_state = RobotState.MANUAL
        self.last_sent_direction = 0
        self.last_sent_pitch = 0.0
        self.last_sent_roll = 0.0
        self.target_direction = 0
        self.filtered_heading = 0.0
        self.last_audio_warning = 0

        # --- Background Serial Worker Thread Management ---
        self.serial_lock = threading.Lock()
        self.gait_update_queue = None
        self.emergency_queue = None  
        self.is_running = True
        
        self.gait_worker_thread = threading.Thread(target=self._gait_serial_worker, daemon=True)
        self.gait_worker_thread.start()

    def init_serial_ports(self):
        """Initializes connection to all 4 leg Picos."""
        for port in self.pico_ports:
            try:
                s = serial.Serial(port, baudrate=115200, timeout=0.1)
                self.ser_list.append(s)
                self.get_logger().info(f"UART setup successful: {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to open {port}: {e}")

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
        for step in angles_matrix:
            flat_angles.extend([step[0], step[1], step[2]])
        msg.data = flat_angles
        self.joint_pub.publish(msg)

    def send_entire_gait(self, angles_list):
        """Hand off the path array safely to the background worker thread."""
        self.publish_joints(angles_list)
        with self.serial_lock:
            self.gait_update_queue = angles_list

    def handle_recovery(self, abort_payload, trigger_serial):
        """Processes recovery calculations safely and hands off execution to the background thread."""
        try:
            parts = abort_payload.split(',')
            curr_roll, curr_pitch, curr_knee = float(parts[1]), float(parts[2]), float(parts[3])
            
            # Calculate current Cartesian coordinates via Forward Kinematics
            start_x, start_y, start_z = self.ik_engine.calculate_fk(curr_roll, curr_pitch, curr_knee)
            
            # Generate path back to neutral home stance
            recovery_gait = self.recovery_engine.get_recovery_gait(start_x, start_y, start_z)

            # Stage details to background worker thread atomically
            with self.serial_lock:
                self.current_state = RobotState.RECOVERY
                self.emergency_queue = (trigger_serial, recovery_gait)
                self.gait_update_queue = None  # Clear outstanding standard gait steps
                
        except Exception as e:
            self.get_logger().error(f"Error in recovery parsing: {e}")

    def _gait_serial_worker(self):
        local_gait = None

        while self.is_running:
            recovery_job = None  # (trigger_serial, recovery_gait) if needed

            with self.serial_lock:
                state_check = self.current_state

                if state_check == RobotState.RECOVERY and self.emergency_queue is not None:
                    recovery_job = self.emergency_queue  # grab it
                    self.emergency_queue = None           # clear it

                elif self.gait_update_queue is not None:
                    local_gait = self.gait_update_queue
                    self.gait_update_queue = None

            # --- Lock is now RELEASED ---

            if recovery_job is not None:
                trigger_serial, recovery_gait = recovery_job
                try:
                    trigger_serial.reset_output_buffer()
                    trigger_serial.write(b'\xAA\xAA')
                    for step in recovery_gait:
                        packed_data = struct.pack('ffff', float(step[0]), float(step[1]), float(step[2]), float(step[3]))
                        trigger_serial.write(packed_data)
                        time.sleep(0.01)  # fine here — lock is not held
                    trigger_serial.write(b'\xFF' * 16)
                except Exception as e:
                    print(f"Serial transmission crash during recovery: {e}")

                with self.serial_lock:
                    self.current_state = RobotState.MANUAL
                local_gait = None
                continue

            if local_gait is None:
                time.sleep(0.005)
                continue
                
            num_steps = len(local_gait)
            offsets = [0, num_steps // 2, (3 * num_steps) // 4, num_steps // 4]
            
            for s in self.ser_list:
                try:
                    s.reset_output_buffer()
                    s.write(b'\xAA\xAA')
                except Exception:
                    pass
                
            aborted = False
            
            # SCOPE 2: Run the physical loop WITHOUT holding the lock globally
            for i in range(num_steps):
                # Briefly check if an emergency abort was triggered by the main thread
                with self.serial_lock:
                    if self.current_state == RobotState.RECOVERY:
                        aborted = True
                        break
                
                for leg_idx, s in enumerate(self.ser_list):
                    step_idx = (i + offsets[leg_idx]) % num_steps
                    step = local_gait[step_idx]
                    packed_data = struct.pack('ffff', float(step[0]), float(step[1]), float(step[2]), float(step[3]))
                    try:
                        s.write(packed_data)
                    except Exception as e:
                        print(f"Serial write error on leg {leg_idx}: {e}")
                
                time.sleep(0.001) # just a brief yield so the lock stays open for the main thread
    
            if aborted:
                local_gait = None 
                continue 
                
            for s in self.ser_list:
                try:
                    s.write(self.end_marker)
                except Exception:
                    pass
                
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
    MAX_CURRENT_MA = 6000.0 
    AUDIO_COOLDOWN = 10.0
    last_power_check = time.time()
    last_audio_warning = 0

    controller = PiQuadrupedController()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(streamer)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("ROS 2 Unified Multi-Node Infrastructure Started")

    controller.send_entire_gait(controller.all_angles)

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
                controller.filtered_heading = (0.15 * raw_head) + (0.85 * controller.filtered_heading)

                with controller.serial_lock:
                    snap_state = controller.current_state
                    snap_target_dir = controller.target_direction
                    snap_last_dir = controller.last_sent_direction
                    snap_last_pitch = controller.last_sent_pitch
                    snap_last_roll = controller.last_sent_roll
                
                try:
                    imu.update() 
                    roll_tilt = imu.get_roll()    
                    pitch_tilt = imu.get_pitch()  
                except Exception as e:
                    print(f"[Hardware Error] Failed to read IMU: {e}")
                    roll_tilt, pitch_tilt = 0.0, 0.0

                Kp_pitch = 0.15  
                Kp_roll = 0.12 
                
                pitch_correction_y = pitch_tilt * Kp_pitch 
                roll_correction_x = roll_tilt * Kp_roll

                if abs(roll_tilt) > 8.0 or abs(pitch_tilt) > 8.0:
                    if int(current_time) % 2 == 0:
                        print(f"[IMU Warning] Large Tilt! Roll: {roll_tilt:.2f}, Pitch: {pitch_tilt:.2f}")

                chosen_direction = snap_target_dir 
                if snap_state == RobotState.AUTONOMOUS:
                    # Safely forward coordinates to the real method name
                    nav_data = nav_engine.calculate_nav(gps.lat, gps.lon, controller.filtered_heading)
                    if nav_data is not None:
                        chosen_direction = nav_data["turn"]

                dir_delta = abs(chosen_direction - snap_last_dir) > 5
                pitch_delta = abs(pitch_tilt - snap_last_pitch) > 1.5
                roll_delta = abs(roll_tilt - snap_last_roll) > 1.5

                if dir_delta or pitch_delta or roll_delta:
                    clipped_pitch_y = max(-3.0, min(3.0, pitch_correction_y))
                    clipped_roll_x = max(-2.5, min(2.5, roll_correction_x))
                    
                    if int(current_time) % 2 == 0: 
                        print(f"[IMU Reflex] Stabilizing active stance matrix. Shifts -> X: {clipped_roll_x:.2f}, Y: {clipped_pitch_y:.2f}")

                    direction_rad = math.radians(chosen_direction)
                    longitudinal_stride = 10.0 * math.cos(direction_rad)  # scales forward stride
                    lateral_stride = 5.0 * math.sin(direction_rad)         # drives lateral_roll_offset

                    controller.path_gen.update_params(
                        center_stride_y=clipped_pitch_y,
                        center_height_z=36.0,
                        length=longitudinal_stride,
                        height1=5.0,
                        height2=2.5,
                        direction_angle=0  # angle no longer used in GaitPath
                    )

                    controller.gait_processor = GaitIK(
                        controller.ik_engine,
                        controller.path_gen.gait_xy_path,
                        lateral_roll_offset=clipped_roll_x + lateral_stride  # IMU roll + steering
                    )
                    new_angles = controller.gait_processor.get_gait_ik()
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

                for s in controller.ser_list:
                    if s.in_waiting > 0:
                        try:
                            line = s.readline().decode('utf-8', errors='ignore').strip()
                            if line.startswith("ABORTED"):
                                print(f"Hardware Stall Warning on UART: {s.port}")
                                audio_engine.play(os.path.join(WAV_DIR, "abort_sound.wav"))
                                controller.handle_recovery(line, s)
                        except Exception as ser_err:
                            print(f"Serial read error: {ser_err}")
                
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