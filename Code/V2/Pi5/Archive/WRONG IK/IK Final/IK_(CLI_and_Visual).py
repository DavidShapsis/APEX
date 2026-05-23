import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# --- CONFIGURATION ---
ROBOT_WIDTH = 12.0
ROBOT_LENGTH = 20.0
FLOOR_Z = -25.0

class RoboLeg:
    def __init__(self, is_left):
        self.is_left = is_left

    def solve_ik(self, x, y, z, a, b, c):
        # 1. ROLL
        d_xz = math.sqrt(x**2 + z**2)
        if d_xz < a: d_xz = a + 0.001
        
        phi = math.atan2(x, -z)
        theta = math.acos(np.clip(a / d_xz, -1, 1))
        roll = phi - theta

        # 2. 2D PROJECTION
        r_reach = math.sqrt(max(0, d_xz**2 - a**2))
        L = math.sqrt(r_reach**2 + y**2)
        L = np.clip(L, 0.01, b + c - 0.001)

        # 3. PITCH & KNEE
        alpha = math.atan2(y, r_reach)
        cos_beta = (b**2 + L**2 - c**2) / (2 * b * L)
        beta = math.acos(np.clip(cos_beta, -1, 1))
        pitch = alpha - beta

        cos_gamma = (b**2 + c**2 - L**2) / (2 * b * c)
        gamma = math.acos(np.clip(cos_gamma, -1, 1))
        knee = math.pi - gamma

        return roll, pitch, knee

    def get_geometry(self, target, shoulder_pos, links):
        tx, ty, tz = target
        a, b, c = links
        side = -1 if self.is_left else 1
        
        roll, pitch, knee = self.solve_ik(tx, ty, tz, a, b, c)
        p0 = np.array(shoulder_pos)

        # FK Logic
        cR, sR = math.cos(roll * side), math.sin(roll * side)
        R_roll = np.array([[cR, 0, sR], [0, 1, 0], [-sR, 0, cR]])
        p1 = p0 + R_roll @ np.array([side * a, 0, 0])

        cP, sP = math.cos(pitch), math.sin(pitch)
        R_pitch = np.array([[1, 0, 0], [0, cP, -sP], [0, sP, cP]])
        p2 = p1 + R_roll @ R_pitch @ np.array([0, 0, -b])

        cK, sK = math.cos(pitch + knee), math.sin(pitch + knee)
        R_knee = np.array([[1, 0, 0], [0, cK, -sK], [0, sK, cK]])
        p3 = p2 + R_roll @ R_knee @ np.array([0, 0, -c])

        return [p0, p1, p2, p3]

class RoboDogController:
    """Handles the logic of the robot without GUI dependencies."""
    def __init__(self):
        w2, l = ROBOT_WIDTH / 2, ROBOT_LENGTH
        self.legs = [RoboLeg(True), RoboLeg(True), RoboLeg(False), RoboLeg(False)]
        self.shoulders = [(-w2,0,0), (-w2,l,0), (w2,l,0), (w2,0,0)]
        self.leg_names = ["Front Left", "Back Left", "Back Right", "Front Right"]

    def calculate_state(self, x, y, z, a, b, c):
        results = []
        target = (x, y, -z)
        links = (a, b, c)
        
        for i, (leg, s_pos) in enumerate(zip(self.legs, self.shoulders)):
            roll, pitch, knee = leg.solve_ik(target[0], target[1], target[2], links[0], links[1], links[2])
            pts = leg.get_geometry(target, s_pos, links)
            results.append({
                "name": self.leg_names[i],
                "angles": (roll, pitch, knee),
                "points": np.array(pts)
            })
        return results

class RoboDogSim:
    """Handles the Matplotlib visualization."""
    def __init__(self, controller):
        self.controller = controller
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(left=0.25, bottom=0.15)
        
        self.elev, self.azim = 20, 45
        
        # Sliders
        ax_bg = '#f0f0f0'
        self.s_z = Slider(plt.axes([0.07, 0.70, 0.12, 0.03], facecolor=ax_bg), 'X (Side)', -5, 10, valinit=2.5)
        self.s_y = Slider(plt.axes([0.07, 0.65, 0.12, 0.03], facecolor=ax_bg), 'Y (Fwd)', -7.5, 7.5, valinit=0.1)
        self.s_x = Slider(plt.axes([0.08, 0.60, 0.12, 0.03], facecolor=ax_bg), 'Z (Down)', 5, 25, valinit=15)
        self.s_a = Slider(plt.axes([0.09, 0.45, 0.12, 0.03], facecolor=ax_bg), 'Seg a', 2.5, 10, valinit=5.0)
        self.s_b = Slider(plt.axes([0.09, 0.40, 0.12, 0.03], facecolor=ax_bg), 'Seg b', 5, 20, valinit=10.0)
        self.s_c = Slider(plt.axes([0.09, 0.35, 0.12, 0.03], facecolor=ax_bg), 'Seg c', 5, 20, valinit=10.0)

        for s in [self.s_x, self.s_y, self.s_z, self.s_a, self.s_b, self.s_c]:
            s.on_changed(self.update_plot)
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.update_plot(None)
        plt.show()

    def update_plot(self, val):
        self.ax.clear()
        # Floor
        for g in np.linspace(-30, 30, 11):
            self.ax.plot([g, g], [-10, 35], [FLOOR_Z, FLOOR_Z], color='#dddddd', lw=0.5)
            self.ax.plot([-30, 30], [g+5, g+5], [FLOOR_Z, FLOOR_Z], color='#dddddd', lw=0.5)

        leg_data = self.controller.calculate_state(self.s_x.val, self.s_y.val, self.s_z.val, 
                                                   self.s_a.val, self.s_b.val, self.s_c.val)
        
        print("\n" + "="*40)
        print(f"INPUTS -> X: {self.s_x.val:.2f}, Y: {self.s_y.val:.2f}, Z: {self.s_z.val:.2f}")
        print("-"*40)

        for data in leg_data:
            pts = data["points"]
            r, p, k = data["angles"]
            # Draw Segments
            self.ax.plot(pts[0:2, 0], pts[0:2, 1], pts[0:2, 2], color='#FF5733', lw=5) 
            self.ax.plot(pts[1:3, 0], pts[1:3, 1], pts[1:3, 2], color='#33FF57', lw=5) 
            self.ax.plot(pts[2:4, 0], pts[2:4, 1], pts[2:4, 2], color='#3357FF', lw=5) 
            self.ax.scatter(pts[:,0], pts[:,1], pts[:,2], color='black', s=30)
            
        print(f"Hip Roll: {math.degrees(r):6.2f} degrees")
        print(f"Hip Pitch: {math.degrees(p):6.2f} degrees")
        print(f"Knee: {math.degrees(k):6.2f} degrees")
        print("-"*40)

        # Body
        w2, l = ROBOT_WIDTH/2, ROBOT_LENGTH
        rect = np.array([[-w2,0,0], [-w2,l,0], [w2,l,0], [w2,0,0], [-w2,0,0]])
        self.ax.plot(rect[:,0], rect[:,1], rect[:,2], 'black', lw=3)
        self.ax.set_xlim(-30, 30); self.ax.set_ylim(-15, 40); self.ax.set_zlim(-30, 10)
        self.ax.view_init(elev=self.elev, azim=self.azim)
        self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key == 'up':    self.elev += 5
        elif event.key == 'down':  self.elev -= 5
        elif event.key == 'left':  self.azim -= 5
        elif event.key == 'right': self.azim += 5
        self.ax.view_init(elev=self.elev, azim=self.azim)
        self.fig.canvas.draw_idle()

# --- MAIN SELECTION ---
if __name__ == "__main__":
    # ----------------------------
    # CHOOSE MODE HERE:
    USE_SIMULATION = False
    # -----------------------------

    dog = RoboDogController()

    if USE_SIMULATION:
        sim = RoboDogSim(dog)
    else:
        results = dog.calculate_state(x=2.5, y=0.1, z=15, a=5, b=10, c=10)
        
        # Note: the below hip roll output when using simulation is off is wrong!
        
        print("\n--- Inverse Kinematics Output ---")
        # Just taking one of the sets of angles cause all 4 legs dont nee dto all be printed out. 
        # I just need x, y, z for various cases and then just using the angles accordingly
        # So, results[0]["anges"] is used
        r, p, k = results[0]["angles"]
        # Pitch is multipled by -1 because for some reason, its negative
        print(f"Hip Roll: {math.degrees(r):.1f}")
        print(f"Hip Pitch: {math.degrees(p)*-1:.2f}")
        print(f"Knee:{math.degrees(k):.2f}")