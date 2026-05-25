import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- COPY OF YOUR LOGIC CLASSES ---
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
        
        hip_roll_rad = math.atan2(x, -z_adj) + math.acos(self._clip(a / r_xz))
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
        h, a = p['len']/2, p['len']/2
        path = []
        for i in range(41):
            theta = (i / 40) * 2 * math.pi + math.pi
            lx = h + a * math.cos(theta)
            ly = (p['h1'] if math.sin(theta) >= 0 else p['h2']) * math.sin(theta)
            rx = lx * math.cos(p['angle']) - ly * math.sin(p['angle'])
            ry = lx * math.sin(p['angle']) + ly * math.cos(p['angle'])
            path.append([p['cx'] + rx, p['cy'] + ry])
        return path

# --- VISUALIZATION SETTINGS ---
SEGMENT_LENGTHS = {'a': 9.65, 'b': 26.84, 'c': 24.37} # in cm
ik = InverseKinematics(SEGMENT_LENGTHS)
# Adjusting center_y to 36 as you set in your last code
gait = GaitPath(2, 36, 10, 5, 2.5, 25)

fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-20, 20)
ax.set_ylim(60, -10) # Inverted Y-axis to match your Z-down logic
ax.set_aspect('equal')
ax.grid(True)
line, = ax.plot([], [], 'o-', lw=4, markersize=10, color='blue')
trail, = ax.plot([], [], '--', alpha=0.3, color='gray')

def update(frame):
    target = gait.path[frame]
    # In your logic: calculate(0, i[0], i[1]) -> x=0, y=coord_x, z=coord_y
    # This means your gait is happening in the Y-Z plane
    r, p, k = ik.calculate(0, target[0], target[1])
    
    # Simple Forward Kinematics for drawing only
    # Shoulder is at (0,0)
    p_rad = math.radians(p)
    k_rad = math.radians(k)
    
    # Knee position
    knee_y = SEGMENT_LENGTHS['b'] * math.sin(p_rad)
    knee_z = SEGMENT_LENGTHS['b'] * math.cos(p_rad)
    
    # Foot position
    foot_y = knee_y + SEGMENT_LENGTHS['c'] * math.sin(p_rad - (math.pi - k_rad))
    foot_z = knee_z + SEGMENT_LENGTHS['c'] * math.cos(p_rad - (math.pi - k_rad))
    
    # X axis in plot = Y in your coordinates, Y axis in plot = Z in your coordinates
    line.set_data([0, knee_y, foot_y], [0, knee_z, foot_z])
    
    # Update trail
    trail_y = [p[0] for p in gait.path]
    trail_z = [p[1] for p in gait.path]
    trail.set_data(trail_y, trail_z)
    
    ax.set_title(f"Gait Frame {frame} | Roll: {r:.1f} Pitch: {p:.1f} Knee: {k:.1f}")
    return line, trail

ani = FuncAnimation(fig, update, frames=len(gait.path), interval=50, blit=True)
plt.show()