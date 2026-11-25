import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# --- IMPORT PRODUCTION CLASSES FROM YOUR CENTRAL FILE ---
try:
    from Code.Pi5.inverse_kinematics.ik_and_gait import InverseKinematics, GaitPath, GaitIK
except ImportError:
    print("\n[ERROR] Could not find 'ik_and_gait.py' in this directory.")
    print("Ensure this script is saved in the same folder as your main codebase.\n")
    raise

# --- INITIALIZE THE PRODUCTION ENGINES ---
ik_engine = InverseKinematics()
gait_generator = GaitPath()

# --- CONFIGURATION: PROGRESSIVE SWEEP & HOLD SETTINGS ---
START_ANGLE = 0
ANGLE_INCREMENT = 15      # Degrees to rotate after holding
STRIDES_PER_ANGLE = 3     # <-- Number of full cycles to execute per angle before switching
current_heading = START_ANGLE

def refresh_trajectory(angle_deg):
    """Updates gait parameters and generates a new set of joint angles."""
    gait_generator.update_params(
        center_stride_y=0, 
        center_height_z=36, 
        length=12, 
        height1=6, 
        height2=1, 
        direction_angle=angle_deg
    )
    gait_processor = GaitIK(ik_engine, gait_generator.gait_xy_path, lateral_roll_offset=0.0)
    return gait_processor.get_gait_ik()

# Generate the initial trajectory
angle_trajectory_list = refresh_trajectory(current_heading)

# --- 3D GRAPHICS WINDOW SETUP ---
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Configure coordinate spaces to look uniform
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(50, -5)  # Positive Z down

ax.set_xlabel('X (Outward / Lateral)')
ax.set_ylabel('Y (Forward / Stride)')
ax.set_zlabel('Z (Downward / Extension)')

# Define visual structural linkages
hip_line, = ax.plot([], [], [], 'o-', lw=4, color='red', label='Segment A (Roll Link)')
leg_line, = ax.plot([], [], [], 'o-', lw=5, color='blue', label='Leg (Pitch/Knee)')
trail_line, = ax.plot([], [], [], '--', alpha=0.6, color='green', label='Foot Track Loop')

# Track total steps and loop index for holding the angle
total_strides_executed = 0
stride_hold_counter = 0

def update(frame):
    global total_strides_executed, stride_hold_counter, angle_trajectory_list, current_heading
    
    # --- ANGLE RETENTION & SWEEP LOGIC ---
    # Intercept when a single 20-frame loop finishes and wraps around to frame 0
    if frame == 0:
        total_strides_executed += 1
        
        # Increment the hold tracking counter
        if total_strides_executed > 1:
            stride_hold_counter += 1
            
            # Once we complete our target number of strides, rotate the compass vector
            if stride_hold_counter >= STRIDES_PER_ANGLE:
                stride_hold_counter = 0  # Reset counter for the next angle
                current_heading = (current_heading + ANGLE_INCREMENT) % 360
                
                # Re-generate the IK path for the new direction vector
                angle_trajectory_list = refresh_trajectory(current_heading)
                
                # Clear visual foot trails so the new path shape doesn't get cluttered
                update.hist_x, update.hist_y, update.hist_z = [], [], []

    # Safe bounds check in case array lengths vary
    if frame >= len(angle_trajectory_list):
        return leg_line, hip_line, trail_line

    # Extract the pre-calculated roll, pitch, knee angles for the current heading
    angles = angle_trajectory_list[frame]
    r_deg, p_deg, k_deg = angles[0], angles[1], angles[2]
    
    # 1. Calculate physical foot location using FK to check math accuracy
    f_x, f_y, f_z = ik_engine.calculate_fk(r_deg, p_deg, k_deg)
    
    # 2. Reconstruct 3D joint nodes matching your specific orientation
    roll_rad = math.radians(r_deg + 90.0)
    pitch_rad = math.radians(p_deg)
    a, b = ik_engine.SEGMENT_LENGTHS['a'], ik_engine.SEGMENT_LENGTHS['b']
    
    s_x = a * math.sin(roll_rad)
    s_y = 0.0
    s_z = a * math.cos(roll_rad)
    
    dist_to_knee = b
    knee_y = dist_to_knee * math.sin(pitch_rad)
    z_rel_knee = dist_to_knee * math.cos(pitch_rad)
    
    r_xz_knee = math.sqrt(z_rel_knee**2 + a**2)
    phi2_k = math.acos(max(-1.0, min(1.0, a / r_xz_knee)))
    phi1_k = roll_rad - phi2_k
    
    k_x = r_xz_knee * math.sin(phi1_k)
    k_y = knee_y
    k_z = r_xz_knee * math.cos(phi1_k)

    # 3. Update physical lines on screen
    hip_line.set_data([0, s_x], [0, s_y])
    hip_line.set_3d_properties([0, s_z])
    
    leg_line.set_data([s_x, k_x, f_x], [s_y, k_y, f_y])
    leg_line.set_3d_properties([s_z, k_z, f_z])
    
    # 4. Handle green trail loop (accumulates cleanly over the 3 repeated strides)
    if not hasattr(update, "hist_x") or len(update.hist_x) == 0: 
        update.hist_x, update.hist_y, update.hist_z = [], [], []
        
    update.hist_x.append(f_x)
    update.hist_y.append(f_y)
    update.hist_z.append(f_z)
    
    trail_line.set_data(update.hist_x, update.hist_y)
    trail_line.set_3d_properties(update.hist_z)

    # Display status metrics inside the window title
    ax.set_title(
        f"Dynamic Compass Steering Test\n"
        f"Target Heading: {current_heading}° | Stride: {stride_hold_counter + 1}/{STRIDES_PER_ANGLE}\n"
        f"Total Strides Tracked: {total_strides_executed}"
    )
    return leg_line, hip_line, trail_line

# Compile and start the animation loop
ani = FuncAnimation(
    fig, 
    update, 
    frames=len(angle_trajectory_list), 
    interval=40, 
    blit=False, 
    cache_frame_data=False
)
plt.legend(loc='upper right')
plt.show()