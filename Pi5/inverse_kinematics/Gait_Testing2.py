import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# --- IMPORT PRODUCTION CLASSES FROM YOUR CENTRAL FILE ---
try:
    from ik_and_gait import InverseKinematics, GaitPath, GaitIK
except ImportError:
    print("\n[ERROR] Could not find 'ik_and_gait.py' in this directory.")
    print("Ensure this script is saved in the same folder as your main codebase.\n")
    raise

# --- INITIALIZE THE PRODUCTION ENGINES ---
ik_engine = InverseKinematics()
gait_generator = GaitPath()

# Configure Stride Parameters: 
# 0cm Y offset, 36cm ground clearance height, 12cm step length,
# 6cm step clearance arc, 1cm ground compression allowance
gait_generator.update_params(
    center_stride_y=0, 
    center_height_z=36, 
    length=12, 
    height1=6, 
    height2=1, 
    direction_angle=0
)

# Process the gait path to generate joint angle sets
gait_processor = GaitIK(ik_engine, gait_generator.gait_xy_path, lateral_roll_offset=0.0)
angle_trajectory_list = gait_processor.get_gait_ik()

# --- 3D GRAPHICS WINDOW SETUP ---
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Configure coordinate spaces to look uniform
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(50, -5)  # Matplotlib 3D Z inverted keeps positive down matching your frame

ax.set_xlabel('X (Outward / Lateral)')
ax.set_ylabel('Y (Forward / Stride)')
ax.set_zlabel('Z (Downward / Extension)')

# Define the visible structural linkages
hip_a_line, = ax.plot([], [], [], 'o-', lw=4, color='red', label='Segment A (Roll Link)')
leg_bc_line, = ax.plot([], [], [], 'o-', lw=5, color='blue', label='Segments B & C (Pitch/Knee)')
trail_line, = ax.plot([], [], [], '--', alpha=0.6, color='green', label='Foot Track Loop')

def update(frame):
    # Extract the pre-calculated roll, pitch, knee angles for the current step frame
    angles = angle_trajectory_list[frame]
    r_deg, p_deg, k_deg = angles[0], angles[1], angles[2]
    
    # 1. Calculate the exact physical foot location using your engine's FK script
    f_x, f_y, f_z = ik_engine.calculate_fk(r_deg, p_deg, k_deg)
    
    # 2. ADAPTIVE FIX: Reconstruct joint nodes using your file's specific +90 deg offset orientation
    roll_rad = math.radians(r_deg + 90.0)
    pitch_rad = math.radians(p_deg)
    
    a, b = ik_engine.SEGMENT_LENGTHS['a'], ik_engine.SEGMENT_LENGTHS['b']
    
    # Shoulder joint location matching your file's standard frame orientation
    s_x = a * math.sin(roll_rad)
    s_y = 0.0
    s_z = a * math.cos(roll_rad)
    
    # Virtual leg translation vectors matching your file's pitch trajectory
    dist_to_knee = b
    knee_y = dist_to_knee * math.sin(pitch_rad)
    z_rel_knee = dist_to_knee * math.cos(pitch_rad)
    
    # Projected 3D Knee Node positions 
    r_xz_knee = math.sqrt(z_rel_knee**2 + a**2)
    phi2_k = math.acos(max(-1.0, min(1.0, a / r_xz_knee)))
    phi1_k = roll_rad - phi2_k
    
    k_x = r_xz_knee * math.sin(phi1_k)
    k_y = knee_y
    k_z = r_xz_knee * math.cos(phi1_k)

    # 3. Update the visual lines on screen
    hip_a_line.set_data([0, s_x], [0, s_y])
    hip_a_line.set_3d_properties([0, s_z])
    
    leg_bc_line.set_data([s_x, k_x, f_x], [s_y, k_y, f_y])
    leg_bc_line.set_3d_properties([s_z, k_z, f_z])
    
    # 4. Handle the green track loop historical trail data arrays
    if not hasattr(update, "hist_x") or frame == 0: 
        update.hist_x, update.hist_y, update.hist_z = [], [], []
        
    update.hist_x.append(f_x)
    update.hist_y.append(f_y)
    update.hist_z.append(f_z)
    
    trail_line.set_data(update.hist_x, update.hist_y)
    trail_line.set_3d_properties(update.hist_z)

    ax.set_title(f"Centralized Robot Leg Gait | Roll: {r_deg:.1f}° | Pitch: {p_deg:.1f}°")
    return leg_bc_line, hip_a_line, trail_line

# Compile and start the loop
ani = FuncAnimation(fig, update, frames=len(angle_trajectory_list), interval=40, blit=False)
plt.legend(loc='upper right')
plt.show()