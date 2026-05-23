# Some notes about what x,y, and z mean in all this
# +x is outward, -x is inward
# +y is forward, -y is backward
# +z is downward, -z is upward  ---- this is the weird maybe a little unintuitive one. esentially, more z is father from the hip

import math

class InverseKinematics:
    def __init__(self, SEGMENT_LENGTHS=None):
        # Using your build's specific cm measurements
        self.SEGMENT_LENGTHS = SEGMENT_LENGTHS if SEGMENT_LENGTHS else {'a': 9.65, 'b': 26.84, 'c': 24.37}
        self.roll = 0.0
        self.pitch = 0.0
        self.knee = 0.0

    def _clip(self, val):
        return max(-1.0, min(1.0, val))

    def calculate(self, x, y, z):
        a, b, c = self.SEGMENT_LENGTHS['a'], self.SEGMENT_LENGTHS['b'], self.SEGMENT_LENGTHS['c']
        
        # Z-down convention: Invert for calculation logic
        z_adj = z * -1

        # 1. Hip Roll - Fixed to flare INWARD
        r_xz = math.sqrt(x**2 + z_adj**2)
        if r_xz < 0.001: r_xz = 0.001
        hip_roll_rad = math.atan2(x, -z_adj) - math.acos(self._clip(a / r_xz))

        # 2. Projection into the Pitch/Knee plane
        z_rel = math.sqrt(max(0, r_xz**2 - a**2))
        y_rel = y
        dist_to_foot_sq = y_rel**2 + z_rel**2
        dist_to_foot = math.sqrt(dist_to_foot_sq)
        if dist_to_foot < 0.001: dist_to_foot = 0.001

        # 3. Hip Pitch
        alpha = math.atan2(y_rel, z_rel)
        beta = math.acos(self._clip((b**2 + dist_to_foot_sq - c**2) / (2 * b * dist_to_foot)))
        hip_pitch_rad = alpha + beta

        # 4. Knee Angle
        knee_angle_rad = math.acos(self._clip((b**2 + c**2 - dist_to_foot_sq) / (2 * b * c)))

        self.roll = math.degrees(hip_roll_rad)
        self.pitch = math.degrees(hip_pitch_rad)
        self.knee = math.degrees(knee_angle_rad)
        
        return self

class GaitPath:
    def __init__(self):
        self.gait_xy_path = []
        self.params = {}

    def update_params(self, center_x, center_y, length, height1, height2, direction_angle):
        self.params = {
            'cx': center_x, 'cy': center_y, 'len': length,
            'h1': height1, 'h2': height2, 'angle': math.radians(direction_angle)
        }
        return self.generate_path()

    def generate_path(self):
        p = self.params
        # Fix: Using p['len']/2 instead of undefined stride_radius
        a = p['len'] / 2    
        angle = p['angle']
        
        self.gait_xy_path = []
        num_steps = 20
        
        for i in range(num_steps + 1):
            # Start at theta=0 for a clean cycle
            theta = (i / num_steps) * 2 * math.pi
            
            # 1. Negative cosine makes Swing go FORWARD
            local_x = -a * math.cos(theta)
            
            # 2. Height h1 (Tall) assigned when sin <= 0 (Upward in Z-down)
            local_y = (p['h1'] if math.sin(theta) <= 0 else p['h2']) * math.sin(theta)

            # 3. Rotate and Translate
            rot_x = local_x * math.cos(angle) - local_y * math.sin(angle)
            rot_y = local_x * math.sin(angle) + local_y * math.cos(angle)
            
            final_x = p['cx'] + rot_x
            final_y = p['cy'] + rot_y
            
            self.gait_xy_path.append([round(float(final_x), 2), round(float(final_y), 2)])
        return self.gait_xy_path

class GaitIK:
    def __init__(self, ik_computer, gait_path):
        self.ik_computer = ik_computer
        self.gait_path = gait_path
        
    def get_gait_ik(self):
        gait_angles_list = []
        last_roll = 0.0
        for i in self.gait_path:
            # i[0] is Forward/Backward, i[1] is Downward
            ik = self.ik_computer.calculate(0, i[0], i[1])
            
            current_roll = ik.roll
            if abs(current_roll - last_roll) > 90:
                current_roll = last_roll
            
            gait_angles_list.append([current_roll, ik.pitch, ik.knee])
            last_roll = current_roll
        return gait_angles_list

class RecoveryPath:
    def __init__(self, ik_computer):
        self.ik_computer = ik_computer
        # Define your "Normal" Home Position
        self.home_x = 0.0   # Center of roll motor as you requested
        self.home_y = 0.0   # Neutral pitch
        self.home_z = 15.0  # Set this to your usual standing height (e.g., 15cm)

    def get_recovery_gait(self, current_x, current_y, current_z, steps=20):
        """
        Generates a list of [roll, pitch, knee] angles to transition 
        from the current position back to home.
        """
        recovery_angles = []
        
        for i in range(steps + 1):
            # Linear interpolation (LERP) formula: start + (end - start) * percentage
            t = i / steps
            
            target_x = current_x + (self.home_x - current_x) * t
            target_y = current_y + (self.home_y - current_y) * t
            target_z = current_z + (self.home_z - current_z) * t
            
            # Calculate IK for this intermediate point
            ik = self.ik_computer.calculate(target_x, target_y, target_z)
            recovery_angles.append([ik.roll, ik.pitch, ik.knee])
            
        return recovery_angles