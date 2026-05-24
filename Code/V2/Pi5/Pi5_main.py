import threading
import time
import serial
import struct
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32, Bool
from smbus2 import SMBus

# Core hardware and engine imports
from Power_Monitor import INA219
from InverseKinematics.IK_and_Gait import InverseKinematics, GaitPath, GaitIK, RecoveryPath
from Audio import QuadrupedAudio
from Webcam import USBWebcam
from Stream_Server import RobodogStreamer
from Navigation import GPSReader, CompassReader, Navigator
from IMU import IMU

print("Imports successful")

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
        
        # --- Hardware Serial Setup ---
        self.pico_ports = ['/dev/ttyAMA0', '/dev/ttyAMA2', '/dev/ttyAMA3', '/dev/ttyAMA4']
        self.ser_list = []
        self.init_serial_ports()
        self.end_marker = b'\xFF' * 12

        # --- Sub-Engine Initializations ---
        self.ik_engine = InverseKinematics({'a': (96.5/10), 'b': (268.404/10), 'c': (243.794/10)})
        self.path_gen = GaitPath()
        self.path_gen.update_params(center_x=5, center_y=36, length=10, height1=5, height2=2.5, direction_angle=0)
        
        self.recovery_engine = RecoveryPath(self.ik_engine)
        self.gait_processor = GaitIK(self.ik_engine, self.path_gen.gait_xy_path)
        self.all_angles = self.gait_processor.get_gait_ik()
        
        # System State Tracking Management
        self.current_state = RobotState.MANUAL
        self.last_sent_direction = 0
        self.target_direction = 0
        self.filtered_heading = 0.0
        self.last_audio_warning = 0

        # --- NEW: Background Serial Worker Thread Management ---
        self.serial_lock = threading.Lock()
        self.gait_update_queue = None
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
        self.target_direction = msg.data

    def publish_joints(self, angles_matrix):
        """Flattens gait matrix and publishes to the ROS world for visualization/logging."""
        msg = Float32MultiArray()
        flat_angles = []
        for step in angles_matrix:
            flat_angles.extend([step[0], step[1], step[2]])
        msg.data = flat_angles
        self.joint_pub.publish(msg)

    def send_entire_gait(self, angles_list):
        """NEW: Hand off the path array safely to the background worker thread."""
        self.publish_joints(angles_list)
        with self.serial_lock:
            self.gait_update_queue = angles_list

    def _gait_serial_worker(self):
        """NEW: Isolated background loop that handles streaming binary chunks to hardware."""
        while self.is_running:
            # Treat recovery mode as an immediate pause flag for normal streaming
            if self.current_state == RobotState.RECOVERY:
                time.sleep(0.05)
                continue
                
            local_gait = None
            
            with self.serial_lock:
                if self.gait_update_queue is not None:
                    local_gait = self.gait_update_queue
                    self.gait_update_queue = None  # Clear queue after grabbing data
            
            if local_gait is None:
                time.sleep(0.005)  # Rest thread if no updates are pending
                continue
                
            num_steps = len(local_gait)
            offsets = [
                0,                      # Leg 0 (Front Left)
                num_steps // 2,         # Leg 1 (Front Right)
                (3 * num_steps) // 4,   # Leg 2 (Back Left)
                num_steps // 4          # Leg 3 (Back Right)
            ]
            
            # Send Binary START Frame Identifier (0xAAAA)
            for s in self.ser_list:
                s.write(b'\xAA\xAA') 
                
            for i in range(num_steps):
                for leg_idx, s in enumerate(self.ser_list):
                    step_idx = (i + offsets[leg_idx]) % num_steps
                    step = local_gait[step_idx]
                    
                    # Pack 3 joint angles into standard binary float bytes (12 bytes total)
                    packed_data = struct.pack('fff', float(step[0]), float(step[1]), float(step[2]))
                    s.write(packed_data)
                    
                time.sleep(0.002) # Safer, faster step pacing delay
                
            # Send Binary END Frame Identifier (0xBBBB)
            for s in self.ser_list:
                s.write(self.end_marker)
    def nav_mode_callback(self, msg):
        """Changes the robot's primary operating state machine channel."""
        if msg.data:
            self.current_state = RobotState.AUTONOMOUS
            self.get_logger().info("Robot State Transited to: AUTONOMOUS_NAV")
        else:
            self.current_state = RobotState.MANUAL
            self.get_logger().info("Robot State Transited to: MANUAL")

    def handle_recovery(self, abort_payload, trigger_serial):
        """Processes recovery calculation and forces state lock."""
        previous_state = self.current_state
        self.current_state = RobotState.RECOVERY
        try:
            parts = abort_payload.split(',')
            curr_roll, curr_pitch, curr_knee = float(parts[1]), float(parts[2]), float(parts[3])
            start_x, start_y, start_z = self.ik_engine.calculate_fk(curr_roll, curr_pitch, curr_knee)
            
            recovery_gait = self.recovery_engine.get_recovery_gait(start_x, start_y, start_z)
            
            # SECURE LOCK so background worker can't mix bytes into this stream
            with self.serial_lock:
                trigger_serial.write(b'\xAA\xAA')
                for step in recovery_gait:
                    packed_data = struct.pack('fff', float(step[0]), float(step[1]), float(step[2]))
                    trigger_serial.write(packed_data)
                trigger_serial.write(b'\xFF' * 12)
            
        except Exception as e:
            self.get_logger().error(f"Error in recovery execution: {e}")
        
        self.current_state = previous_state

    def close_hardware(self):
        """Gracefully closes all hardware serial lines."""
        self.is_running = False
        for s in self.ser_list:
            s.close()

# --- Companion Peripheral Loops & Initializations ---
rclpy.init(args=None)

# IMU Configuration
imu = IMU(sda_pin="D0", scl_pin="D1", bus_id=13, window_size=12)
print("IMU setup successful")

# Navigation Configuration
MISSION_WAYPOINTS = [(41.056, -74.145), (41.057, -74.146)] 
gps = GPSReader(uart_path='/dev/ttyUSB0', baudrate=9600)
compass = CompassReader(bus_id=1)     
nav_engine = Navigator(MISSION_WAYPOINTS)

# Vision (Instantiated ONCE globally so the thread targets the correct object)
cam = USBWebcam(device_index=0)
streamer = RobodogStreamer()

def camera_loop():
    while True:
        frame = cam.get_frame()
        if frame is not None:
            streamer.update_frame(frame)
        time.sleep(0.03)

# Start background vision components
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


# --- Execution Runtime Entrypoint ---
def main():
    controller = PiQuadrupedController()

    # Create a MultiThreadedExecutor to manage both nodes cleanly under one system context
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(streamer)

    # Spin the unified executor in the background
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("ROS 2 Unified Multi-Node Infrastructure Started")

    # Fire initial baseline gait trajectory
    controller.send_entire_gait(controller.all_angles)

    try:
        while rclpy.ok():
            current_time = time.time()
            gps.update()
            
            # 1. Read & Filter Compass Orientation Data
            raw_head = compass.get_heading()
            controller.filtered_heading = (0.1 * raw_head) + (0.9 * controller.filtered_heading)

            # 2. Extract IMU Readings for Closed-Loop Stability Correction
            roll_tilt = imu.get_roll()    # Left-to-right lean
            pitch_tilt = imu.get_pitch()  # Front-to-back lean
            
            if abs(roll_tilt) > 8.0 or abs(pitch_tilt) > 8.0:
                print(f"[IMU Warning] Tilt detected! Roll: {roll_tilt:.2f}, Pitch: {pitch_tilt:.2f}")

            # 3 & 4. Evaluate States and Extract Navigation Directives
            chosen_direction = controller.last_sent_direction
            
            if controller.current_state == RobotState.MANUAL:
                chosen_direction = controller.target_direction
                
            elif controller.current_state == RobotState.AUTONOMOUS:
                if gps.has_fix:
                    nav_data = nav_engine.calculate_nav(gps.lat, gps.lon, controller.filtered_heading)
                    if nav_data:
                        chosen_direction = nav_data['turn']
                        streamer.current_direction = chosen_direction
                        if int(current_time) % 2 == 0: 
                            print(f"[AUTO] Target Turn: {chosen_direction:.1f}° | Dist: {nav_data['dist']:.1f}m")
                    else:
                        print("Destination Reached! Returning to Manual Hold.")
                        # Publish a direct message to alter the global state cleanly
                        msg = Bool()
                        msg.data = False
                        controller.nav_mode_pub.publish(msg) # Let the subscription callbacks sync the state uniformly!
                else:
                    if int(current_time) % 5 == 0:
                        print("[AUTO Warning] Waiting for valid GPS Fix...")

            elif controller.current_state == RobotState.RECOVERY:
                # Stall logic handles writing movements directly; bypass routine kinematic loop changes
                time.sleep(0.01)
                continue

            # Check if steering direction has updated significantly
            if abs(chosen_direction - controller.last_sent_direction) > 5:
                print(f"Recalculating Kinematics Path for Direction: {chosen_direction}°")
                controller.path_gen.update_params(
                    center_x=5, center_y=36, length=10, height1=5, height2=2.5, direction_angle=chosen_direction
                )
                controller.gait_processor = GaitIK(controller.ik_engine, controller.path_gen.gait_xy_path)
                new_angles = controller.gait_processor.get_gait_ik()
                
                controller.send_entire_gait(new_angles)
                controller.last_sent_direction = chosen_direction

            # 5. Diagnostic Power Threshold Checks
            if current_time - last_power_check > 1.0:
                v = power_monitor.get_voltage()
                c = power_monitor.get_current()
                if (v < LOW_VOLT_THRESHOLD or c > MAX_CURRENT_MA) and (current_time - last_audio_warning > AUDIO_COOLDOWN):
                    audio_engine.play("low_battery.wav")
                    # Fixed local variable assignment scope issue inside main()
                    last_audio_warning = current_time
                last_power_check = current_time

            # 6. Physical Microcontroller Stall Monitoring
            for s in controller.ser_list:
                if s.in_waiting > 0:
                    try:
                        line = s.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("ABORTED"):
                            print(f"Hardware Stall Warning on UART: {s.port}")
                            audio_engine.play("abort_sound.wav")
                            controller.handle_recovery(line, s)
                    except Exception as ser_err:
                        print(f"Serial read error: {ser_err}")
            
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