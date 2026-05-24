import math

class InverseKinematics:
    def __init__(self, SEGMENT_LENGTHS=None):
        # Standardized segment lengths in cm
        self.SEGMENT_LENGTHS = SEGMENT_LENGTHS if SEGMENT_LENGTHS else {'a': 9.65, 'b': 26.84, 'c': 24.37}
        self.roll = 0.0
        self.pitch = 0.0
        self.knee = 0.0

    def _clip(self, val):
        return max(-1.0, min(1.0, val))

    def calculate(self, x, y, z):
        """
        Calculates joint angles from standardized target coordinates:
        x: Lateral offset (+ right, - left)
        y: Stride displacement (+ forward, - backward)
        z: Extension height (+ down)
        """
        a, b, c = self.SEGMENT_LENGTHS['a'], self.SEGMENT_LENGTHS['b'], self.SEGMENT_LENGTHS['c']
        
        # 1. Roll calculation in the X-Z plane
        r_xz = math.sqrt(x**2 + z**2)
        if r_xz < a:
            r_xz = a
        
        phi1 = math.atan2(x, z)
        phi2 = math.acos(self._clip(a / r_xz))
        self.roll = math.degrees(phi1 + phi2) - 90.0 
        
        # 2. Pitch and Knee calculation using virtual leg length in the Y-Z plane
        z_rel = math.sqrt(max(0, r_xz**2 - a**2))
        d_sq = y**2 + z_rel**2
        d = math.sqrt(d_sq)
        
        cos_knee = (b**2 + c**2 - d_sq) / (2 * b * c)
        self.knee = math.degrees(math.acos(self._clip(cos_knee)))
        
        cos_beta = (b**2 + d_sq - c**2) / (2 * b * d)
        beta = math.acos(self._clip(cos_beta))
        alpha = math.atan2(y, z_rel)
        
        self.pitch = math.degrees(alpha + beta)
        return self

    def calculate_fk(self, hip_roll_deg, hip_pitch_deg, knee_deg):
        """Calculates X, Y, Z position from joint angles matching the standardized frame."""
        roll_rad = math.radians(hip_roll_deg + 90.0)
        pitch_rad = math.radians(hip_pitch_deg)
        knee_rad = math.radians(knee_deg)
        
        a, b, c = self.SEGMENT_LENGTHS['a'], self.SEGMENT_LENGTHS['b'], self.SEGMENT_LENGTHS['c']
        
        dist_to_foot_sq = b**2 + c**2 - 2 * b * c * math.cos(knee_rad)
        dist_to_foot = math.sqrt(max(0, dist_to_foot_sq))
        
        beta = math.acos(self._clip((b**2 + dist_to_foot_sq - c**2) / (2 * b * dist_to_foot)))
        alpha = pitch_rad - beta
        
        y = dist_to_foot * math.sin(alpha)
        z_rel = dist_to_foot * math.cos(alpha)
        
        r_xz = math.sqrt(max(0, z_rel**2 + a**2))
        
        # Reconstruct coordinates to match standard frame orientation
        phi2_fk = math.acos(self._clip(a / r_xz))
        phi1_fk = roll_rad - phi2_fk
        x = r_xz * math.sin(phi1_fk)
        z = r_xz * math.cos(phi1_fk)
        
        return round(x, 2), round(y, 2), round(z, 2)

class GaitIK:
    def __init__(self, ik_computer, gait_path, lateral_roll_offset=0.0):
        self.ik_computer = ik_computer
        self.gait_path = gait_path
        self.lateral_roll_offset = lateral_roll_offset
        
    def get_gait_ik(self):
        gait_angles_list = []
        last_roll = 0.0
        for i in self.gait_path:
            # i[0] is now final_x, i[1] is final_y, i[2] is final_z, i[3] is is_swing
            # We combine the base lateral offset (like chassis width) with the step deflection
            target_x = self.lateral_roll_offset + i[0]
            
            ik = self.ik_computer.calculate(x=target_x, y=i[1], z=i[2])
            is_swing = i[3] if len(i) > 3 else False
            
            current_roll = ik.roll
            if abs(current_roll - last_roll) > 90:
                current_roll = last_roll
            
            gait_angles_list.append([current_roll, ik.pitch, ik.knee, 1.0 if is_swing else 0.0])
            last_roll = current_roll
        return gait_angles_list

class GaitPath:
    def __init__(self):
        self.gait_xy_path = []
        self.params = {}

    def update_params(self, center_stride_y, center_height_z, length, height1, height2, direction_angle):
        self.params = {
            'cy': center_stride_y, 'cz': center_height_z, 'len': length,
            'h1': height1, 'h2': height2, 'angle': math.radians(direction_angle)
        }
        return self.generate_path()

    def generate_path(self):
        p = self.params
        half_len = p['len'] / 2
        self.gait_xy_path = []
        num_steps = 20
        for i in range(num_steps):
            theta = (i / num_steps) * 2 * math.pi
            
            # --- THE FLIP: Positive half_len * cos(theta) ---
            # This matches +Y (Forward) during the swing phase (when sin(theta) > 0)
            stride_magnitude = half_len * math.cos(theta)
            
            # Use direction_angle to project the stride onto X and Y axes
            local_x = stride_magnitude * math.sin(p['angle'])  
            local_y = stride_magnitude * math.cos(p['angle'])  
            
            # Vertical trajectory component (Positive sin(theta) means lifting UP)
            local_z_raw = (p['h1'] if math.sin(theta) >= 0 else p['h2']) * math.sin(theta)
            
            # Add base offsets 
            final_x = local_x  
            final_y = p['cy'] + local_y
            final_z = p['cz'] - local_z_raw  # Subtracting lifts the leg up
            
            # Swing flag tracks when the leg is lifting forward through the air
            is_swing = math.sin(theta) > 0.0
            
            self.gait_xy_path.append([
                round(float(final_x), 2), 
                round(float(final_y), 2), 
                round(float(final_z), 2), 
                is_swing
            ])
        return self.gait_xy_path

class RecoveryPath:
    def __init__(self, ik_computer):
        self.ik_computer = ik_computer
        self.home_x = 0.0   
        self.home_y = 0.0   
        self.home_z = 36.0  

    def get_recovery_gait(self, current_x, current_y, current_z, steps=20):
        """Generates structural trajectory back to home stance coordinates."""
        recovery_angles = []
        for i in range(steps + 1):
            t = i / steps
            target_x = current_x + (self.home_x - current_x) * t
            target_y = current_y + (self.home_y - current_y) * t
            target_z = current_z + (self.home_z - current_z) * t
            
            ik = self.ik_computer.calculate(x=target_x, y=target_y, z=target_z)
            recovery_angles.append([ik.roll, ik.pitch, ik.knee, 0.0])
            
        return recovery_angles