import math
import matplotlib.pyplot as plt
import numpy as np

# --- GLOBAL CONFIGURATION ---
ROBOT_WIDTH = 12.0   
ROBOT_LENGTH = 20.0  
LINKS = {'a': 5.0, 'b': 9.0, 'c': 9.0}

class RoboLeg:
    def __init__(self, name, is_left):
        self.a, self.b, self.c = LINKS['a'], LINKS['b'], LINKS['c']
        self.name = name
        self.is_left = is_left

    def _clip(self, val):
        return max(-1.0, min(1.0, val))

    def solve_ik(self, x, y, z):
        """
        IK for a single leg. 
        X is 'outward' from the shoulder, Y is Forward, Z is Up.
        """
        r_xz = math.sqrt(x**2 + z**2)
        phi = math.atan2(x, -z)
        theta = math.asin(self._clip(self.a / r_xz))
        
        roll = phi - theta
        x_rel = math.sqrt(max(0, r_xz**2 - self.a**2))
        dist_sq = x_rel**2 + y**2
        dist = math.sqrt(dist_sq)

        # Backward facing knees
        alpha = math.atan2(y, x_rel)
        beta = math.acos(self._clip((self.b**2 + dist_sq - self.c**2) / (2 * self.b * dist)))
        pitch = alpha - beta 

        knee_angle = math.acos(self._clip((self.b**2 + self.c**2 - dist_sq) / (2 * self.b * self.c)))
        return roll, pitch, knee_angle

    def get_geometry(self, target_relative, shoulder_pos):
        # Target relative: x is 'outward', y is 'forward', z is 'up'
        tx, ty, tz = target_relative
        sx, sy, sz = shoulder_pos
        
        roll, pitch, knee = self.solve_ik(tx, ty, tz)

        # J0: Shoulder
        j0 = np.array([sx, sy, sz])
        
        # MIRROR LOGIC:
        # On the right side, positive roll swings the leg 'out' (positive X).
        # On the left side, positive roll must swing the leg 'out' (negative X).
        side_mult = -1 if self.is_left else 1
        
        # J1: End of Hip Offset (Orange)
        # We use side_mult to ensure the 'a' segment projects correctly
        j1 = j0 + np.array([
            side_mult * self.a * math.cos(roll), 
            0, 
            -self.a * math.sin(roll)
        ])
        
        # J2: Knee (Green)
        k_x_local = (self.a + self.b * math.cos(pitch)) * math.cos(roll)
        k_y_local = self.b * math.sin(pitch)
        k_z_local = -(self.a + self.b * math.cos(pitch)) * math.sin(roll)
        
        j2 = j0 + np.array([side_mult * k_x_local, k_y_local, k_z_local])
        
        # J3: Foot (Blue)
        # World position of the foot
        j3 = j0 + np.array([side_mult * tx, ty, tz])
        
        return [j0, j1, j2, j3]

class QuadrupedVisualizer:
    def __init__(self):
        self.elev, self.azim, self.dist = 25, 45, 20

    def on_key(self, event):
        if event.key == 'up':    self.elev += 5
        elif event.key == 'down':  self.elev -= 5
        elif event.key == 'left':  self.azim -= 5
        elif event.key == 'right': self.azim += 5
        elif event.key == 'w':     self.dist = max(5, self.dist - 1)
        elif event.key == 's':     self.dist += 1
        self.update_view()

    def update_view(self):
        self.ax.view_init(elev=self.elev, azim=self.azim)
        self.ax.dist = self.dist
        self.fig.canvas.draw_idle()

    def plot(self, legs_list):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        colors = ['#FF5733', '#33FF57', '#3357FF'] # Orange, Green, Blue

        for leg_obj, target_rel, shoulder_pos in legs_list:
            pts = np.array(leg_obj.get_geometry(target_rel, shoulder_pos))
            
            # Draw segments with individual colors
            self.ax.plot(pts[0:2, 0], pts[0:2, 1], pts[0:2, 2], color=colors[0], linewidth=4, marker='o')
            self.ax.plot(pts[1:3, 0], pts[1:3, 1], pts[1:3, 2], color=colors[1], linewidth=4, marker='o')
            self.ax.plot(pts[2:4, 0], pts[2:4, 1], pts[2:4, 2], color=colors[2], linewidth=4, marker='o')

        # Chassis outline
        w2, l = ROBOT_WIDTH/2, ROBOT_LENGTH
        rect = np.array([[-w2,0,0], [-w2,l,0], [w2,l,0], [w2,0,0], [-w2,0,0]])
        self.ax.plot(rect[:,0], rect[:,1], rect[:,2], 'k--', alpha=0.3)

        self.ax.set_box_aspect([1, 1, 1]) 
        self.ax.set_xlim(-20, 20); self.ax.set_ylim(-5, 25); self.ax.set_zlim(-20, 5)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.update_view()
        plt.show()

if __name__ == "__main__":
    w2 = ROBOT_WIDTH / 2
    
    # Initialize 4 legs with their specific side property
    legs = [
        (RoboLeg("Back Left",   is_left=True),  (2, 0, -12), (-w2, 0, 0)),
        (RoboLeg("Front Left",  is_left=True),  (2, 0, -12), (-w2, ROBOT_LENGTH, 0)),
        (RoboLeg("Front Right", is_left=False), (2, 0, -12), ( w2, ROBOT_LENGTH, 0)),
        (RoboLeg("Back Right",  is_left=False), (2, 0, -12), ( w2, 0, 0))
    ]
    
    viz = QuadrupedVisualizer()
    viz.plot(legs)