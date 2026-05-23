import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
from smbus2 import SMBus
import threading
from Power_Monitor import INA219
from InverseKinematics.IK_and_Gait import InverseKinematics, GaitPath, GaitIK, RecoveryPath
from Audio import QuadrupedAudio
from Webcam import USBWebcam
from Stream_Server import RobodogStreamer
from Navigation import GPSReader, CompassReader, Navigator
from IMU import IMU # Actually IMPLEMENT THE IMU AND UTILIZE IT FOR CORRECTING WRONG MOVEMENTS!
print("Imports successful")

class PiMainRosBridge(Node):
    def __init__(self):
        super().__init__('pi5_main_node')
        # Create a publisher for joint targets
        self.joint_pub = self.create_publisher(Float32MultiArray, '/apex/kinematics/joint_targets', 10)
        
    def publish_joints(self, angles_matrix):
        """Helper to flatten your gait matrix and publish it to ROS"""
        msg = Float32MultiArray()
        flat_angles = []
        for step in angles_matrix:
            flat_angles.extend([step[0], step[1], step[2]])
        msg.data = flat_angles
        self.joint_pub.publish(msg)

# -- UART Serial Config Setup (4 Ports) --
pico_ports = ['/dev/ttyAMA0', '/dev/ttyAMA2', '/dev/ttyAMA3', '/dev/ttyAMA4']
ser_list = []

for port in pico_ports:
    try:
        s = serial.Serial(port, baudrate=115200, timeout=0.1)
        ser_list.append(s)
        print(f"UART setup successful: {port}")
    except Exception as e:
        print(f"Failed to open {port}: {e}")

# --- IMU Setup ---
imu = IMU(sda_pin="D0", scl_pin="D1", bus_id=13, window_size=12)
print("IMU setup successful")

# --- Navigation Setup ---
# Example Waypoints (Lat, Lon)
MISSION_WAYPOINTS = [(41.056, -74.145), (41.057, -74.146)] 
gps = GPSReader(uart_path='/dev/ttyUSB0', baudrate=9600) # Using your 5th UART port
compass = CompassReader(bus_id=1)     # Shared I2C bus with INA219
nav_engine = Navigator(MISSION_WAYPOINTS)

NAV_MODE = False # Toggle this to switch between Manual (Flask) and Auto (GPS)

# -- IK and Gait Setup --
ik_engine = InverseKinematics({'a': (96.5/10), 'b': (268.404/10), 'c': (243.794/10)})
path_gen = GaitPath()
path_gen.update_params(center_x=5, center_y=36, length=10, height1=5, height2=2.5, direction_angle=0)

recovery_engine = RecoveryPath(ik_engine)
gait_processor = GaitIK(ik_engine, path_gen.gait_xy_path)
all_angles = gait_processor.get_gait_ik()
print("IK and Gait setup successful")

# --- Vision Initialization ---
cam = USBWebcam(device_index=0)
streamer = RobodogStreamer()
def start_stream():
    streamer.run() # This stays in its own thread

stream_thread = threading.Thread(target=start_stream, daemon=True)
stream_thread.start()
print("Vision setup successful")

# --- Initialize ROS 2 Bridge ---
rclpy.init(args=None)
ros_node = PiMainRosBridge()

# Spin the node in a background thread so rclpy.spin() doesn't block our main loop
ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
ros_thread.start()
print("ROS 2 Background Bridge successfully started")

# --- Power Monitor Setup ---
LOW_VOLT_THRESHOLD = 4.75
MAX_CURRENT_MA = 6000.0 
AUDIO_COOLDOWN = 10.0 

power_monitor = INA219(bus_id=3)
last_power_check = time.time()
power_check_interval = 1.0 
print("Power Monitor setup successful")

# --- Audio Engine Setup ---
SPEAKER_MAC = "30:8D:EB:5D:AC:11"
audio_engine = QuadrupedAudio(SPEAKER_MAC) 
last_audio_warning = 0
print("Audio Engine setup successful")

# --- Camera Background Task ---
def camera_loop():
    while True:
        frame = cam.get_frame()
        if frame is not None:
            streamer.update_frame(frame)
        time.sleep(0.03)

cam_thread = threading.Thread(target=camera_loop, daemon=True)
cam_thread.start()

# -- Modified for 4 Picos --
def send_entire_gait(angles_list):
    """Sends the gait sequence to all 4 Picos with phase offsets."""
    ros_node.publish_joints(angles_list)
    
    num_steps = len(angles_list)
    
    # One-at-a-time (Stable Crawl) offsets:
    # Sequence: FL (0) -> BR (25%) -> FR (50%) -> BL (75%)
    offsets = [
        0,                      # Leg 0 (Front Left)
        num_steps // 2,         # Leg 1 (Front Right) - Starts at 50%
        (3 * num_steps) // 4,   # Leg 2 (Back Left)   - Starts at 75%
        num_steps // 4          # Leg 3 (Back Right)  - Starts at 25%
    ]

    # Example Offsets for a trot gait - Leg 0 & 3 move together; Leg 1 & 2 move together (50% shift)
    # offsets = [0, num_steps//2, num_steps//2, 0]
    
    for s in ser_list:
        s.write(b"START\n")
    
    # We loop through time slices, sending 1 coordinate to each leg per slice
    for i in range(num_steps):
        for leg_idx, s in enumerate(ser_list):
            # Calculate this leg's current position in the pattern
            step_idx = (i + offsets[leg_idx]) % num_steps
            step = angles_list[step_idx]
            payload = f"{step[0]:.2f},{step[1]:.2f},{step[2]:.2f}\n"
            s.write(payload.encode('utf-8'))
        time.sleep(0.01) # Small delay to prevent UART buffer overflow
            
    for s in ser_list:
        s.write(b"END\n")

def handle_recovery(abort_payload, trigger_serial):
    """Handles recovery for the specific Pico that aborted."""
    try:
        parts = abort_payload.split(',')
        curr_roll, curr_pitch, curr_knee = float(parts[1]), float(parts[2]), float(parts[3])
        start_x, start_y, start_z = ik_engine.calculate_fk(curr_roll, curr_pitch, curr_knee)
        
        recovery_gait = recovery_engine.get_recovery_gait(start_x, start_y, start_z)
        
        # Send recovery specifically to the pico that complained
        trigger_serial.write(b"START\n")
        for step in recovery_gait:
            payload = f"{step[0]:.2f},{step[1]:.2f},{step[2]:.2f}\n"
            trigger_serial.write(payload.encode('utf-8'))
        trigger_serial.write(b"END\n")
        
    except Exception as e:
        print(f"Error in recovery: {e}")

last_sent_direction = 0
filtered_heading = 0.0

# -- Main Loop --
try:
    send_entire_gait(all_angles)

    while True:
        current_time = time.time()
    
        # 1. Update Sensors
        gps.update()
        
        # --- SMOOTHING FILTER START ---
        raw_head = compass.get_heading()
        # The filter: 10% new data, 90% old data
        filtered_heading = (0.1 * raw_head) + (0.9 * filtered_heading)
        # --- SMOOTHING FILTER END ---

        if NAV_MODE and gps.has_fix:
            # Use the SMOOTHED heading for navigation math
            nav_data = nav_engine.calculate_nav(gps.lat, gps.lon, filtered_heading)
            
            if nav_data:
                streamer.current_direction = nav_data['turn']
                # Only print occasionally to avoid spamming the console
                if int(current_time) % 2 == 0: 
                    print(f"GPS Nav -> Dist: {nav_data['dist']:.1f}m, Turn: {nav_data['turn']:.1f}°")
            else:
                print("Mission Complete!")
                NAV_MODE = False

        # 2. Check for direction change
        # Using the smoothed direction prevents the IK from recalculating constantly
        if abs(streamer.current_direction - last_sent_direction) > 5:
            new_dir = streamer.current_direction
            print(f"New Course Correction: {new_dir}°")
            
            path_gen.update_params(center_x=5, center_y=36, length=10, height1=5, height2=2.5, direction_angle=new_dir)
            gait_processor = GaitIK(ik_engine, path_gen.gait_xy_path)
            new_angles = gait_processor.get_gait_ik()
            
            send_entire_gait(new_angles)
            last_sent_direction = new_dir

        # 3. Power Check
        if current_time - last_power_check > power_check_interval:
            v = power_monitor.get_voltage()
            c = power_monitor.get_current()
            last_power_check = current_time

            if v < LOW_VOLT_THRESHOLD or c > MAX_CURRENT_MA:
                if current_time - last_audio_warning > AUDIO_COOLDOWN:
                    audio_engine.play("low_battery.wav")
                    last_audio_warning = current_time

        # 4. Check all 4 UARTs for Aborts
        for s in ser_list:
            if s.in_waiting > 0:
                line = s.readline().decode('utf-8').strip()
                if line.startswith("ABORTED"):
                    print(f"Abort detected on {s.port}")
                    audio_engine.play("abort_sound.wav")
                    handle_recovery(line, s)
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[!] Shutdown initiated by user...")
    for s in ser_list:
        s.close()
    cam.release()
    
    # Clean up ROS 2
    ros_node.destroy_node()
    rclpy.shutdown()