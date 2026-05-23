import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# --- GLOBAL CONFIG ---
ROBOT_WIDTH = 12.0
ROBOT_LENGTH = 20.0
FLOOR_Z = -25.0

class RoboLeg:
    def __init__(self, is_left):
        self.is_left = is_left

    def solve_ik(self, x, y, z, a, b, c):
        # 1. ROLL (Handles Sideways X and Vertical Z)
        # We need to find the angle that puts the leg plane through the target
        # while accounting for the hip offset 'a'.
        
        # Distance in the X-Z plane
        d_xz = math.sqrt(x**2 + z**2)
        if d_xz < a: d_xz = a + 0.001 # Physical limit safety
        
        # Base angle to target
        phi = math.atan2(x, -z)
        # Angle offset for the link 'a' triangle
        # This is where the "90 degree corner" usually happens if not clipped
        theta = math.acos(np.clip(a / d_xz, -1, 1))
        
        roll = phi - theta

        # 2. 2D PROJECTION (Inside the rotated leg plane)
        # Reach in the leg plane (radial distance from hip pivot)
        # This is the 'horizontal' reach of the b/c links
        r_reach = math.sqrt(max(0, d_xz**2 - a**2))
        
        # Reach in the 'y' direction (forward/back)
        # Total distance the b and c links must span
        L = math.sqrt(r_reach**2 + y**2)
        L = np.clip(L, 0.01, b + c - 0.001) # Clamp to max physical reach

        # 3. PITCH & KNEE (Law of Cosines)
        # Angle of the target vector in the leg plane
        alpha = math.atan2(y, r_reach)
        
        # Angle of the thigh (b) relative to that vector
        cos_beta = (b**2 + L**2 - c**2) / (2 * b * L)
        beta = math.acos(np.clip(cos_beta, -1, 1))
        
        pitch = alpha - beta

        # Knee angle (0 is straight, pi is folded)
        cos_gamma = (b**2 + c**2 - L**2) / (2 * b * c)
        gamma = math.acos(np.clip(cos_gamma, -1, 1))
        knee = math.pi - gamma

        return roll, pitch, knee

    def get_geometry(self, target, shoulder_pos, links):
        tx, ty, tz = target
        a, b, c = links
        side = -1 if self.is_left else 1
        
        roll, pitch, knee = self.solve_ik(tx, ty, tz, a, b, c)

        # FK: Build the leg using FIXED link lengths
        p0 = np.array(shoulder_pos)

        # Rotation matrix for Roll (around Y axis)
        cR, sR = math.cos(roll * side), math.sin(roll * side)
        R_roll = np.array([[cR, 0, sR], [0, 1, 0], [-sR, 0, cR]])

        # Link A: Fixed distance 'a' out from shoulder
        p1 = p0 + R_roll @ np.array([side * a, 0, 0])

        # Combined Rotation for Pitch (Roll then Local Pitch)
        # We rotate around the new local X axis
        cP, sP = math.cos(pitch), math.sin(pitch)
        R_pitch = np.array([[1, 0, 0], [0, cP, -sP], [0, sP, cP]])
        
        # Link B: Length 'b' down from p1
        p2 = p1 + R_roll @ R_pitch @ np.array([0, 0, -b])

        # Link C: Length 'c' down from p2 (Pitch + Knee)
        cK, sK = math.cos(pitch + knee), math.sin(pitch + knee)
        R_knee = np.array([[1, 0, 0], [0, cK, -sK], [0, sK, cK]])
        p3 = p2 + R_roll @ R_knee @ np.array([0, 0, -c])

        return [p0, p1, p2, p3]

class RoboDogSim:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(left=0.25, bottom=0.15)
        
        self.elev, self.azim = 20, 45
        w2, l = ROBOT_WIDTH / 2, ROBOT_LENGTH
        self.legs = [RoboLeg(True), RoboLeg(True), RoboLeg(False), RoboLeg(False)]
        self.shoulders = [(-w2,0,0), (-w2,l,0), (w2,l,0), (w2,0,0)]

        # --- SLIDERS ---
        ax_bg = '#f0f0f0'
        # Local X (Side), Y (Forward), Z (Height/Vertical)
        self.s_z = Slider(plt.axes([0.05, 0.70, 0.12, 0.03], facecolor=ax_bg), 'X (Side)', -25, -5, valinit=-2.5)
        self.s_y = Slider(plt.axes([0.05, 0.65, 0.12, 0.03], facecolor=ax_bg), 'Y (Fwd)', -15, 15, valinit=0.1)
        self.s_x = Slider(plt.axes([0.05, 0.60, 0.12, 0.03], facecolor=ax_bg), 'Z (Hgt)', 0.1, 30.0, valinit=15)
        
        self.s_a = Slider(plt.axes([0.05, 0.45, 0.12, 0.03], facecolor=ax_bg), 'Link a', 1, 10, valinit=5.0)
        self.s_b = Slider(plt.axes([0.05, 0.40, 0.12, 0.03], facecolor=ax_bg), 'Link b', 5, 20, valinit=10.0)
        self.s_c = Slider(plt.axes([0.05, 0.35, 0.12, 0.03], facecolor=ax_bg), 'Link c', 5, 20, valinit=10.0)

        for s in [self.s_x, self.s_y, self.s_z, self.s_a, self.s_b, self.s_c]:
            s.on_changed(self.update_plot)
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.update_plot(None)
        plt.show()

    def on_key(self, event):
        if event.key == 'up':    self.elev += 5
        elif event.key == 'down':  self.elev -= 5
        elif event.key == 'left':  self.azim -= 5
        elif event.key == 'right': self.azim += 5
        self.update_view()

    def update_view(self):
        self.ax.view_init(elev=self.elev, azim=self.azim)
        self.fig.canvas.draw_idle()

    def update_plot(self, val):
        self.ax.clear()
        target = (self.s_x.val, self.s_y.val, self.s_z.val)
        links = (self.s_a.val, self.s_b.val, self.s_c.val)
        
        # Floor
        for g in np.linspace(-30, 30, 11):
            self.ax.plot([g, g], [-10, 35], [FLOOR_Z, FLOOR_Z], color='#dddddd', lw=0.5)
            self.ax.plot([-30, 30], [g+5, g+5], [FLOOR_Z, FLOOR_Z], color='#dddddd', lw=0.5)

        for leg, s_pos in zip(self.legs, self.shoulders):
            pts = np.array(leg.get_geometry(target, s_pos, links))
            
            # Leg Segments: Orange (a), Green (b), Blue (c)
            self.ax.plot(pts[0:2, 0], pts[0:2, 1], pts[0:2, 2], color='#FF5733', lw=5) 
            self.ax.plot(pts[1:3, 0], pts[1:3, 1], pts[1:3, 2], color='#33FF57', lw=5) 
            self.ax.plot(pts[2:4, 0], pts[2:4, 1], pts[2:4, 2], color='#3357FF', lw=5) 
            self.ax.scatter(pts[:,0], pts[:,1], pts[:,2], color='black', s=30)
            
            # # Target Proof: Red cross where the foot MUST land
            # side = -1 if leg.is_left else 1
            # world_t = np.array(s_pos) + np.array([target[0]*side, target[1], target[2]])
            # self.ax.scatter(world_t[0], world_t[1], world_t[2], color='r', marker='x', s=100)

        # Body
        w2, l = ROBOT_WIDTH/2, ROBOT_LENGTH
        rect = np.array([[-w2,0,0], [-w2,l,0], [w2,l,0], [w2,0,0], [-w2,0,0]])
        self.ax.plot(rect[:,0], rect[:,1], rect[:,2], 'black', lw=3)

        self.ax.set_xlim(-30, 30); self.ax.set_ylim(-15, 40); self.ax.set_zlim(-30, 10)
        self.update_view()

if __name__ == "__main__":
    sim = RoboDogSim()