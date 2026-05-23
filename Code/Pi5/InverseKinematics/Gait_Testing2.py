import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

class InverseKinematics:
    def __init__(self, SEGMENT_LENGTHS):
        self.SEGMENT_LENGTHS = SEGMENT_LENGTHS
    
    def _clip(self, val):
        return max(-1.0, min(1.0, val))

    def calculate(self, x, y, z):
        a, b, c = self.SEGMENT_LENGTHS['a'], self.SEGMENT_LENGTHS['b'], self.SEGMENT_LENGTHS['c']
        z_adj = z * -1 

        r_xz = math.sqrt(x**2 + z_adj**2)
        if r_xz < 0.001: r_xz = 0.001
        
        # --- THE FIX: Subtract acos to point the leg INWARD ---
        hip_roll_rad = math.atan2(x, -z_adj) - math.acos(self._clip(a / r_xz))

        z_rel = math.sqrt(max(0, r_xz**2 - a**2))
        y_rel = y
        dist_to_foot_sq = y_rel**2 + z_rel**2
        dist_to_foot = math.sqrt(dist_to_foot_sq)
        
        alpha = math.atan2(y_rel, z_rel)
        beta = math.acos(self._clip((b**2 + dist_to_foot_sq - c**2) / (2 * b * dist_to_foot)))
        hip_pitch_rad = alpha + beta
        knee_angle_rad = math.acos(self._clip((b**2 + c**2 - dist_to_foot_sq) / (2 * b * c)))

        return math.degrees(hip_roll_rad), math.degrees(hip_pitch_rad), math.degrees(knee_angle_rad)

class GaitPath:
    def __init__(self, cx, cy, length, h1, h2, direction_angle):
        self.params = {'cx': cx, 'cy': cy, 'len': length, 'h1': h1, 'h2': h2, 'angle': math.radians(direction_angle)}
        self.path = self.generate()

    def generate(self):
        p = self.params
        stride_radius = p['len']/2
        path = []
        num_steps = 40
        for i in range(num_steps + 1):
            theta = (i / num_steps) * 2 * math.pi
            # Forward while Swing, Backward while Stance
            local_x = -stride_radius * math.cos(theta)
            # High Arc while Swing (Negative Z), Low Arc while Stance (Positive Z)
            local_y = (p['h1'] if math.sin(theta) <= 0 else p['h2']) * math.sin(theta)
            
            rx = local_x * math.cos(p['angle']) - local_y * math.sin(p['angle'])
            ry = local_x * math.sin(p['angle']) + local_y * math.cos(p['angle'])
            path.append([p['cx'] + rx, p['cy'] + ry])
        return path

# --- 3D VISUALIZATION ---
SEGMENT_LENGTHS = {'a': 9.65, 'b': 26.84, 'c': 24.37}
ik_engine = InverseKinematics(SEGMENT_LENGTHS)
gait_gen = GaitPath(0, 36, 12, 6, 1, 0) # 6cm step height, 1cm ground compression

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-20, 20); ax.set_ylim(-20, 20); ax.set_zlim(60, -10)
ax.set_xlabel('X (Outward)'); ax.set_ylabel('Y (Forward)'); ax.set_zlabel('Z (Downward)')

hip_a_line, = ax.plot([], [], [], 'o-', lw=4, color='red', label='Segment A (Roll)')
leg_bc_line, = ax.plot([], [], [], 'o-', lw=5, color='blue', label='Segments B & C (Pitch/Knee)')
trail_line, = ax.plot([], [], [], '--', alpha=0.5, color='green', label='Foot Path')

def update(frame):
    target = gait_gen.path[frame]
    tx, ty, tz = 0, target[0], target[1]
    r_deg, p_deg, k_deg = ik_engine.calculate(tx, ty, tz)
    
    r, p, k = math.radians(r_deg), math.radians(p_deg), math.radians(k_deg)
    a, b, c = SEGMENT_LENGTHS['a'], SEGMENT_LENGTHS['b'], SEGMENT_LENGTHS['c']
    
    # 1. Shoulder Position
    s_x, s_y, s_z = a * math.cos(r), 0, a * math.sin(r)
    # 2. Knee Position (Orthogonal to A)
    knee_y, z_dist_p = b * math.sin(p), b * math.cos(p)
    k_x, k_y, k_z = s_x - z_dist_p * math.sin(r), s_y + knee_y, s_z + z_dist_p * math.cos(r)
    # 3. Foot Position
    foot_pitch_angle = p - (math.pi - k)
    f_y_rel, z_dist_f = c * math.sin(foot_pitch_angle), c * math.cos(foot_pitch_angle)
    f_x, f_y, f_z = k_x - z_dist_f * math.sin(r), k_y + f_y_rel, k_z + z_dist_f * math.cos(r)

    hip_a_line.set_data([0, s_x], [0, 0]); hip_a_line.set_3d_properties([0, s_z])
    leg_bc_line.set_data([s_x, k_x, f_x], [0, k_y, f_y]); leg_bc_line.set_3d_properties([s_z, k_z, f_z])
    
    if not hasattr(update, "tx") or frame == 0: update.tx, update.ty, update.tz = [], [], []
    update.tx.append(f_x); update.ty.append(f_y); update.tz.append(f_z)
    trail_line.set_data(update.tx, update.ty); trail_line.set_3d_properties(update.tz)

    ax.set_title(f"Final 3D Gait | Foot X Error: {abs(f_x):.4f}")
    return leg_bc_line, hip_a_line, trail_line

ani = FuncAnimation(fig, update, frames=len(gait_gen.path), interval=50, blit=False)
plt.legend(); plt.show()