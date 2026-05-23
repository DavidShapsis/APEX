import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# --- GLOBAL CONFIG ---
ROBOT_WIDTH = 12.0
ROBOT_LENGTH = 20.0
FLOOR_Z = -20.0

class RoboLeg:
    def __init__(self, name, is_left):
        self.name = name
        self.is_left = is_left

    def _clip(self, val):
        return max(-1.0, min(1.0, val))

    def get_rotation_matrix(self, axis, theta):
        """Returns a 3x3 rotation matrix for a given axis and angle."""
        c, s = math.cos(theta), math.sin(theta)
        if axis == 'x':
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif axis == 'y':
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        elif axis == 'z':
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return np.eye(3)

    def solve_ik(self, x, y, z, a, b, c):
        """
        Solves Inverse Kinematics for a 3-DOF Leg.
        Returns: roll, pitch, knee (radians)
        """
        # 1. ROLL (Rotation around Y-axis)
        # Project target onto X-Z plane to solve for the Hip Roll.
        # This handles the lateral 'outward' distance.
        dist_xz = math.sqrt(x**2 + z**2)
        if dist_xz < a: dist_xz = a  # Physical limit check
        
        # Calculate the roll angle required to place the hip pivot correctly
        # The 'a' link sits horizontally when roll is 0.
        phi = math.atan2(x, -z)
        theta = math.acos(self._clip(a / dist_xz))
        roll = phi - theta

        # 2. PITCH & KNEE (Planar 2-Link IK)
        # We now project the problem into the plane of the leg.
        # 'x_rel' is the distance from the hip pivot to the target in the radial direction.
        x_rel = math.sqrt(max(0, dist_xz**2 - a**2))
        
        # Now we solve a simple 2D triangle:
        # Base: x_rel (radial), Height: y (forward)
        reach_sq = x_rel**2 + y**2
        reach = math.sqrt(reach_sq)
        
        # Clamp reach so we don't break the math if target is too far
        max_len = b + c
        if reach > max_len: 
            reach = max_len
            reach_sq = reach**2

        # Law of Cosines for Pitch (Shoulder angle in the leg plane)
        # angle 'alpha' is the angle of the target vector
        alpha = math.atan2(y, x_rel)
        # angle 'beta' is the internal angle of the triangle at the shoulder
        cos_beta = (b**2 + reach_sq - c**2) / (2 * b * reach)
        beta = math.acos(self._clip(cos_beta))
        
        # Pitch: rotation around Local X
        pitch = alpha - beta

        # Law of Cosines for Knee (Elbow angle)
        cos_knee = (b**2 + c**2 - reach_sq) / (2 * b * c)
        knee_internal = math.acos(self._clip(cos_knee))
        
        # Convert internal triangle angle to servo rotation angle
        # We want "Dog Leg" style: Knee bends so the foot comes closer.
        # Standard convention: 0 is straight, pi is folded back.
        knee = math.pi - knee_internal

        return roll, pitch, knee

    def get_geometry(self, target_local, shoulder_pos, links):
        lx, ly, lz = target_local
        sx, sy, sz = shoulder_pos
        a, b, c = links
        
        # 1. Get Angles from IK
        roll, pitch, knee = self.solve_ik(lx, ly, lz, a, b, c)

        # 2. Forward Kinematics (Drawing the leg using Matrices)
        
        # Base transformation (Shoulder position)
        p0 = np.array([sx, sy, sz])
        
        # Define orientation of the side (Left vs Right)
        # For the left side, we mirror the World X inputs.
        side_sign = -1 if self.is_left else 1
        
        # MATRIX CHAIN:
        # We calculate points relative to p0 using rotation matrices.
        
        # R_roll: Rotates the entire leg plane around the Y axis
        R_roll = self.get_rotation_matrix('y', roll * side_sign) 
        # Note: We multiply roll by side_sign because 'outward' roll is opposite for L/R
        
        # p1: End of Link A (Hip Pivot)
        # Link A points strictly OUTWARD (Local X)
        vec_a = np.array([side_sign * a, 0, 0])
        p1 = p0 + R_roll @ vec_a
        
        # R_pitch: Rotates Link B around the Local X axis
        # Note: Because we rotated the frame with R_roll, the "Local X" axis 
        # has moved. But mathematically, we just chain the matrices.
        # For the Pitch, a positive rotation moves the leg Forward (if axis is +X).
        R_pitch = self.get_rotation_matrix('x', pitch)
        
        # p2: End of Link B (Knee)
        # Link B points DOWN (Local -Z) in its default pose (pitch=0)
        vec_b = np.array([0, 0, -b])
        # Chain: Roll -> Pitch -> Vector
        # We apply side_sign to the pitch rotation axis frame if needed, 
        # but usually pitch moves identical for both sides in local frame.
        p2 = p1 + R_roll @ R_pitch @ vec_b
        
        # R_knee: Rotates Link C relative to Link B
        # The knee rotation adds to the pitch rotation
        R_knee = self.get_rotation_matrix('x', knee)
        
        # p3: End of Link C (Foot)
        vec_c = np.array([0, 0, -c])
        # Chain: Roll -> (Pitch + Knee) -> Vector
        # Note: Rotations sum up because they are around the same axis (Local X)
        R_total_leg = self.get_rotation_matrix('x', pitch + knee)
        
        p3 = p2 + R_roll @ R_total_leg @ vec_c
        
        return [p0, p1, p2, p3]

class RoboDogSim:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(left=0.25, bottom=0.15)
        
        self.elev, self.azim, self.dist = 25, 45, 18
        w2 = ROBOT_WIDTH / 2
        
        self.legs = [
            RoboLeg("BL", True), RoboLeg("FL", True),
            RoboLeg("FR", False), RoboLeg("BR", False)
        ]
        self.shoulders = [(-w2,0,0), (-w2,ROBOT_LENGTH,0), (w2,ROBOT_LENGTH,0), (w2,0,0)]

        # UI Sliders
        ax_bg = '#f0f0f0'
        # Targets are LOCAL: X=Outward, Y=Forward, Z=Down
        self.s_x = Slider(plt.axes([0.05, 0.70, 0.12, 0.03], facecolor=ax_bg), 'Local X', 0.1, 15, valinit=3.0)
        self.s_y = Slider(plt.axes([0.05, 0.65, 0.12, 0.03], facecolor=ax_bg), 'Local Y', -10, 10, valinit=0.0)
        self.s_z = Slider(plt.axes([0.05, 0.60, 0.12, 0.03], facecolor=ax_bg), 'Local Z', -25, -5, valinit=-15.0)
        
        self.s_a = Slider(plt.axes([0.05, 0.45, 0.12, 0.03], facecolor=ax_bg), 'Link a', 1, 10, valinit=5)
        self.s_b = Slider(plt.axes([0.05, 0.40, 0.12, 0.03], facecolor=ax_bg), 'Link b', 5, 15, valinit=9)
        self.s_c = Slider(plt.axes([0.05, 0.35, 0.12, 0.03], facecolor=ax_bg), 'Link c', 5, 15, valinit=9)

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
        elif event.key == 'w':     self.dist = max(5, self.dist - 1)
        elif event.key == 's':     self.dist += 1
        self.update_view()

    def update_view(self):
        self.ax.view_init(elev=self.elev, azim=self.azim)
        self.ax.dist = self.dist
        self.fig.canvas.draw_idle()

    def update_plot(self, val):
        self.ax.clear()
        target_local = (self.s_x.val, self.s_y.val, self.s_z.val)
        links = (self.s_a.val, self.s_b.val, self.s_c.val)
        
        # Draw Floor
        for g in np.linspace(-30, 30, 11):
            self.ax.plot([g, g], [-10, 35], [FLOOR_Z, FLOOR_Z], color='#cccccc', lw=0.5)
            self.ax.plot([-30, 30], [g+5, g+5], [FLOOR_Z, FLOOR_Z], color='#cccccc', lw=0.5)

        for leg, s_pos in zip(self.legs, self.shoulders):
            pts = np.array(leg.get_geometry(target_local, s_pos, links))
            
            # Plot Segments
            # Hip Link (Orange)
            self.ax.plot(pts[0:2, 0], pts[0:2, 1], pts[0:2, 2], color='#FF5733', lw=5, solid_capstyle='round')
            # Thigh Link (Green)
            self.ax.plot(pts[1:3, 0], pts[1:3, 1], pts[1:3, 2], color='#33FF57', lw=5, solid_capstyle='round')
            # Calf Link (Blue)
            self.ax.plot(pts[2:4, 0], pts[2:4, 1], pts[2:4, 2], color='#3357FF', lw=5, solid_capstyle='round')
            
            # Draw joints
            self.ax.scatter(pts[:,0], pts[:,1], pts[:,2], color='k', s=25)
            
            # Shadow
            self.ax.plot(pts[:, 0], pts[:, 1], [FLOOR_Z]*4, color='black', alpha=0.1)

        # Draw Chassis
        w2, l = ROBOT_WIDTH/2, ROBOT_LENGTH
        rect = np.array([[-w2,0,0], [-w2,l,0], [w2,l,0], [w2,0,0], [-w2,0,0]])
        self.ax.plot(rect[:,0], rect[:,1], rect[:,2], 'k-', lw=3)
        # Front indicator
        self.ax.text(0, l, 0, "FRONT", color='k', ha='center')

        self.ax.set_box_aspect([1, 1, 1])
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-15, 40)
        self.ax.set_zlim(-25, 10)
        self.update_view()

if __name__ == "__main__":
    sim = RoboDogSim()