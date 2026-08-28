import math

# =============================================================================
# ROBOT CONFIGURATION -- single source of truth, imported by pi5_main.py and
# quadruped_sim.py so the two cannot drift apart.
# =============================================================================

LEG_FL, LEG_FR, LEG_RR, LEG_RL = 0, 1, 2, 3
LEG_ORDER = (LEG_FL, LEG_FR, LEG_RR, LEG_RL)
LEG_NAMES = {LEG_FL: 'Front-Left', LEG_FR: 'Front-Right',
             LEG_RR: 'Rear-Right', LEG_RL: 'Rear-Left'}

LEFT_LEGS = (LEG_FL, LEG_RL)
# The IK solves a knee-forward leg, matching how the rear pair is mounted. The
# front pair is bolted on turned round (knees back, as on a real dog), so its
# stride is mirrored to keep all four feet pushing backward together.
FRONT_LEGS = (LEG_FL, LEG_FR)
# Diagonally opposite corner -- the direction to lean when a leg lifts.
OPPOSITE_LEG = {LEG_FL: LEG_RR, LEG_RR: LEG_FL, LEG_FR: LEG_RL, LEG_RL: LEG_FR}

# Mechanical reference pose for zeroing each leg's encoder, driven one leg at a
# time. Derived, not guessed, from the description: abductor perpendicular
# (roll=90), knee locked straight (knee=180), leg perpendicular to the body's
# lengthwise axis. With the knee locked, thigh and shin are colinear so the
# knee-side interior angle (`beta` in calculate_fk) is exactly 0 regardless of
# segment lengths -- which forces pitch=0 to be the *exact* angle with zero
# fore/aft foot offset, not an approximation. Verified against calculate_fk:
# y comes out to 0.0000 cm. Every leg is commanded the same standardized angles
# here; the per-joint `reverse` flags are what make that swing out to each
# leg's own correct physical side.
HOME_POSE = (90.0, 0.0, 180.0)

# Hip-pivot spacings in cm. The shell is 950mm end to end, but the hip pitch
# axes are 675mm apart -- the support polygon is built from hips, so this must
# be the hip spacing, not the outer length.
BODY_LENGTH_CM = 67.5
BODY_WIDTH_CM = 53.0

# Hip position in the body frame: +X right, +Y forward.
HIP_POSITIONS = {
    LEG_FL: (-BODY_WIDTH_CM / 2, +BODY_LENGTH_CM / 2),
    LEG_FR: (+BODY_WIDTH_CM / 2, +BODY_LENGTH_CM / 2),
    LEG_RR: (+BODY_WIDTH_CM / 2, -BODY_LENGTH_CM / 2),
    LEG_RL: (-BODY_WIDTH_CM / 2, -BODY_LENGTH_CM / 2),
}

# How each leg's local frame maps into the body frame. Left legs are mirror-image
# assemblies so their local +X points left; front legs are turned round so their
# local +Y points aft. Multiply a body-frame vector by these to command it.
LEG_SIGN_X = {leg: (-1.0 if leg in LEFT_LEGS else 1.0) for leg in LEG_ORDER}
LEG_SIGN_Y = {leg: (-1.0 if leg in FRONT_LEGS else 1.0) for leg in LEG_ORDER}


def phase_offsets(num_steps):
    """Per-leg phase offsets, indexed by leg id.

    Evenly spaced quarter-cycle, which walks the legs in the stable crawl order
    FL -> RR -> FR -> RL with a four-feet-down beat between each lift.
    """
    return [0, num_steps // 2, (3 * num_steps) // 4, num_steps // 4]


def airborne_schedule(swing_steps, offsets, num_steps):
    """Which leg is off the ground at each global tick, or None if none is.

    Returns None for a tick where zero or more than one leg is airborne -- the
    latter means the gait is not a valid crawl and callers should notice.
    """
    swing = set(swing_steps)
    schedule = []
    for tick in range(num_steps):
        up = [leg for leg in LEG_ORDER if (tick + offsets[leg]) % num_steps in swing]
        schedule.append(up[0] if len(up) == 1 else None)
    return schedule


def _circular_smooth(points, width):
    """Box filter over a periodic sequence of (x, y)."""
    if width <= 1:
        return list(points)
    n = len(points)
    half = width // 2
    out = []
    for i in range(n):
        window = [points[(i + k - half) % n] for k in range(width)]
        out.append((sum(p[0] for p in window) / width,
                    sum(p[1] for p in window) / width))
    return out


def body_shift_profile(swing_steps, offsets, num_steps, magnitude, smooth_width=1):
    """Where the body should sit, per global tick, so it does not tip mid-crawl.

    With the feet directly under the hips, lifting one leg leaves a support
    triangle whose hypotenuse runs corner to corner THROUGH the body centre --
    the centre is on the edge by construction and the static margin is ~4 mm.
    Leaning toward the diagonally opposite corner before each lift buys real
    margin: 2 cm of shift takes the worst case to +2.4 cm.

    smooth_width defaults to 1 (no smoothing) deliberately. Box-filtering this
    signal was tried and made things worse, not better: the target flips sign at
    each lift boundary, which is also the exact instant the margin is thinnest,
    so any window wider than one tick averages toward zero right when the lean is
    needed most. Measured effect at magnitude=2.0: width 1 gives +2.4 cm worst
    case; width 5 (25% of the cycle) gives only +0.8 cm. The step itself is not a
    problem for the controller -- it is the same order of discontinuity the gait
    already has at every liftoff/touchdown.

    Returns [(sx, sy)] in body-frame cm, one entry per tick. Positive X is right,
    positive Y is forward. The feet must move by the NEGATIVE of this.
    """
    if magnitude <= 0:
        return [(0.0, 0.0)] * num_steps

    schedule = airborne_schedule(swing_steps, offsets, num_steps)

    # On all-four-down ticks, lean toward wherever the NEXT lift needs the body,
    # so the weight has already transferred by the time the foot leaves.
    filled = list(schedule)
    for i in range(num_steps):
        if filled[i] is None:
            for step in range(1, num_steps + 1):
                nxt = schedule[(i + step) % num_steps]
                if nxt is not None:
                    filled[i] = nxt
                    break

    raw = []
    for leg in filled:
        if leg is None:                      # no valid crawl -- do not shift
            raw.append((0.0, 0.0))
            continue
        hx, hy = HIP_POSITIONS[OPPOSITE_LEG[leg]]
        norm = math.hypot(hx, hy)
        raw.append((magnitude * hx / norm, magnitude * hy / norm))

    return _circular_smooth(raw, smooth_width)


def apply_body_shift(path, leg_id, offsets, shift_profile, num_steps):
    """Folds the body sway into one leg's foot path, in that leg's local frame.

    The body moving by S means every planted foot moves by -S relative to it.
    Leg L plays buffer index j at global tick (j - offset_L), so each leg samples
    the profile at its own phase -- which is why the four legs no longer share a
    single rotated buffer once sway is switched on.
    """
    out = []
    for j, step in enumerate(path):
        sx, sy = shift_profile[(j - offsets[leg_id]) % num_steps]
        out.append([step[0] - sx * LEG_SIGN_X[leg_id],
                    step[1] - sy * LEG_SIGN_Y[leg_id],
                    step[2], step[3]])
    return out


def body_twist_xy_path(hip_xy, sign_x, sign_y, num_steps=20, swing_fraction=0.25,
                       fwd_cm=10.0, yaw_rad=0.0, center_height_z=36.0,
                       swing_height=5.0, stance_push=0.0):
    """Per-tick ``[x, y, z, is_swing]`` foot trajectory in one leg's LOCAL frame
    for a gait that advances the body ``fwd_cm`` and yaws it ``yaw_rad``
    (CCW / turning left is positive) per full cycle.

    This is the turning mechanism. ``GaitPath.direction_angle`` rotated every
    leg's stride by the *same* angle, which strafes the body diagonally without
    ever yawing it (verified in simulation). Here each planted foot instead
    arcs about the BODY CENTRE: its neutral position (the hip, ``hip_xy`` in the
    body frame) is rotated by ``+/- yaw_rad/2`` across the stroke and shifted
    ``+/- fwd_cm/2`` forward, so the four feet phased together drive the body
    through both a translation and a rotation. Feet on the outside of the turn
    sweep a longer arc; on a pure spin (``fwd_cm == 0``) the inside feet sweep
    backward. The lateral (x) component of that arc is what recruits the
    hip-roll joint into the turn.

    ``sign_x`` / ``sign_y`` are the leg's mount reflections (``LEG_SIGN_X`` /
    ``LEG_SIGN_Y``): body = hip + local * sign, so local = (body - hip) * sign.

    At ``yaw_rad == 0`` the output is identical, term for term, to the old
    ``GaitPath`` straight gait -- the swing half-ellipse and linear stance are
    unchanged, only re-expressed through ``frac``.
    """
    hx, hy = hip_xy
    beta = max(0.05, min(0.95, swing_fraction))
    out = []
    for i in range(num_steps):
        phase = i / num_steps

        if phase < beta:
            # SWING -- eased forward sweep + half-sine lift (unchanged shape).
            s = phase / beta
            frac = 0.5 - 0.5 * math.cos(math.pi * s)   # 0 (rear) -> 1 (front)
            lift = swing_height * math.sin(math.pi * s)
        else:
            # STANCE -- linear travel back, so every planted foot moves at one
            # rate and they do not scrub against each other.
            s = (phase - beta) / (1.0 - beta)
            frac = 1.0 - s                              # 1 (front) -> 0 (rear)
            lift = -stance_push * math.sin(math.pi * s)

        # Foot position in the BODY frame: neutral hip position rotated by the
        # body's share of the yaw and offset by its share of the forward step,
        # both centred on neutral (+/- half each way).
        ang = yaw_rad * (frac - 0.5)
        fwd = fwd_cm * (frac - 0.5)
        bx = hx * math.cos(ang) - hy * math.sin(ang)
        by = hx * math.sin(ang) + hy * math.cos(ang) + fwd

        local_x = (bx - hx) * sign_x
        local_y = (by - hy) * sign_y
        local_z = center_height_z - lift

        out.append([
            round(float(local_x), 2),
            round(float(local_y), 2),
            round(float(local_z), 2),
            lift > 0.0,
        ])
    return out


def attitude_height_offsets(roll_deg, pitch_deg, gain=0.6, limit_cm=4.0,
                            max_tilt_deg=30.0):
    """Per-leg foot-height change that levels the body. Returns {leg: dz_cm}.

    This is DIFFERENTIAL, which is the whole point: raising the low side and
    lowering the high side produces a real restoring moment. Applying one common
    offset to all four legs -- as the original code did -- only translates the
    body and can never correct attitude, whatever the gain.

    Larger z means a more extended leg, which pushes that corner of the body up.
    A positive roll is taken as right-side-down and a positive pitch as nose-up;
    if the BNO085 is mounted with either axis inverted, flip the sign here rather
    than anywhere downstream.
    """
    r = math.radians(max(-max_tilt_deg, min(max_tilt_deg, roll_deg)))
    p = math.radians(max(-max_tilt_deg, min(max_tilt_deg, pitch_deg)))
    out = {}
    for leg in LEG_ORDER:
        hx, hy = HIP_POSITIONS[leg]
        dz = gain * (hx * math.tan(r) - hy * math.tan(p))
        out[leg] = max(-limit_cm, min(limit_cm, dz))
    return out


class IKSolution:
    """One solve's answer, independent of the engine that produced it.

    calculate() used to store the result on the engine and return `self`, which
    is only safe while a single thread is solving. pi5_main shares ONE
    InverseKinematics between the main control loop (build_gait) and the ROS
    executor threads (stand/go callbacks -> build_ramp -> cartesian_ramp), so a
    concurrent solve could overwrite roll/pitch/knee between the caller getting
    the object back and reading the angles off it -- silently handing one
    thread the other thread's joint targets, which then go straight to a motor.
    Measured with the GIL switch interval forced low: 60% of solves corrupted.

    Attribute names match what every call site already reads, so this is a
    drop-in replacement for the old `return self`.
    """

    __slots__ = ('roll', 'pitch', 'knee')

    def __init__(self, roll, pitch, knee):
        self.roll = roll
        self.pitch = pitch
        self.knee = knee

    def as_tuple(self):
        return (self.roll, self.pitch, self.knee)

    def __repr__(self):
        return (f"IKSolution(roll={self.roll:.3f}, pitch={self.pitch:.3f}, "
                f"knee={self.knee:.3f})")


class InverseKinematics:
    """Stateless solver -- safe to share between threads.

    Holds only the segment lengths, which are read-only after construction.
    Everything a solve produces comes back in an IKSolution.
    """

    def __init__(self, SEGMENT_LENGTHS=None):
        # Standardized segment lengths in cm
        self.SEGMENT_LENGTHS = SEGMENT_LENGTHS if SEGMENT_LENGTHS else {'a': 9.65, 'b': 26.84, 'c': 24.37}

    def _clip(self, val):
        return max(-1.0, min(1.0, val))

    def calculate(self, x, y, z):
        """
        Calculates joint angles from standardized target coordinates:
        x: Lateral offset (+ right, - left)
        y: Stride displacement (+ forward, - backward)
        z: Extension height (+ down)

        Returns an IKSolution. Every intermediate is a local, so concurrent
        solves on the same engine cannot interfere -- see IKSolution.
        """
        a, b, c = self.SEGMENT_LENGTHS['a'], self.SEGMENT_LENGTHS['b'], self.SEGMENT_LENGTHS['c']

        # 1. Roll calculation in the X-Z plane
        r_xz = math.sqrt(x**2 + z**2)
        if r_xz < a:
            r_xz = a

        phi1 = math.atan2(x, z)
        phi2 = math.acos(self._clip(a / r_xz))
        roll = math.degrees(phi1 + phi2) - 90.0

        # 2. Pitch and Knee calculation using virtual leg length in the Y-Z plane
        z_rel = math.sqrt(max(0, r_xz**2 - a**2))
        d = math.sqrt(y**2 + z_rel**2)

        # Clamp into the reachable annulus: the foot can be no closer to the
        # shoulder than |b-c| (leg fully folded) nor further than b+c (fully
        # extended). A no-op anywhere in the normal workspace, but it stops
        # d == 0 dividing by zero below -- which any target inside the abductor
        # length reaches, and which the recovery path can interpolate through.
        d = max(abs(b - c), min(b + c, d))
        d_sq = d * d

        cos_knee = (b**2 + c**2 - d_sq) / (2 * b * c)
        knee = math.degrees(math.acos(self._clip(cos_knee)))

        cos_beta = (b**2 + d_sq - c**2) / (2 * b * d)
        beta = math.acos(self._clip(cos_beta))
        alpha = math.atan2(y, z_rel)

        pitch = math.degrees(alpha + beta)
        return IKSolution(roll, pitch, knee)

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

    def update_params(self, center_stride_y, center_height_z, length, height1, height2,
                      direction_angle, swing_fraction=0.25, mirror_y=False):
        """
        swing_fraction: portion of the cycle the foot spends in the air. 0.25 leaves
            the foot planted for the other 75%, which is what a one-leg-at-a-time
            crawl needs -- three feet are always down. 0.5 would be a trot.
        mirror_y: negates the stride direction for legs mounted facing the opposite
            way (the rear pair, whose knees point forward). This is a spatial mirror,
            not a time reversal, so it leaves the leg's lift timing untouched.
        """
        self.params = {
            'cy': center_stride_y, 'cz': center_height_z, 'len': length,
            'h1': height1, 'h2': height2, 'angle': math.radians(direction_angle),
            'swing': swing_fraction, 'mirror': mirror_y
        }
        return self.generate_path()

    def generate_path(self):
        p = self.params
        half_len = p['len'] / 2
        beta = max(0.05, min(0.95, p['swing']))
        self.gait_xy_path = []
        num_steps = 20
        for i in range(num_steps):
            phase = i / num_steps

            if phase < beta:
                # SWING -- airborne, covering the whole stride forward quickly.
                # A true half-ellipse: y and lift are the cos/sin of the same
                # angle, so the foot sweeps a smooth arc over the ground and eases
                # into liftoff and touchdown instead of slamming into them.
                s = phase / beta
                stride_magnitude = -half_len * math.cos(math.pi * s)
                lift = p['h1'] * math.sin(math.pi * s)
            else:
                # STANCE -- planted, travelling backward over the rest of the cycle.
                # y is LINEAR in time here on purpose, not elliptical: three feet
                # are on the ground at different points in this phase, and a foot
                # pinned to the ground cannot change speed. Give stance a cosine
                # and the planted feet travel at up to 3x each other's rate, so
                # they scrub and fight instead of driving the body cleanly.
                s = (phase - beta) / (1.0 - beta)
                stride_magnitude = half_len - p['len'] * s
                lift = -p['h2'] * math.sin(math.pi * s)

            if p['mirror']:
                stride_magnitude = -stride_magnitude

            # Use direction_angle to project the stride onto X and Y axes
            local_x = stride_magnitude * math.sin(p['angle'])
            local_y = stride_magnitude * math.cos(p['angle'])

            # Add base offsets
            final_x = local_x
            final_y = p['cy'] + local_y
            final_z = p['cz'] - lift  # Subtracting lifts the leg up

            # The flag means "foot should be clear of the ground", which is what the
            # Pico's FSR abort tests against. Liftoff and touchdown sit exactly at
            # neutral height, so they stay unflagged -- otherwise the legitimate
            # ground contact at those instants trips a false abort every cycle.
            is_swing = lift > 0.0

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


def cartesian_ramp(ik_computer, start_angles, target_angles, steps=40):
    """One-shot trajectory between two joint poses, interpolated in Cartesian
    space rather than angle space, so the foot travels a straight line instead
    of an arbitrary curve. Same pattern as RecoveryPath, generalized to any
    target instead of a fixed home stance.

    Used for the boot-time ramp from HOME_POSE into the walking gait's first
    step, so the legs ease into motion instead of the PID being asked to close
    a ~45-95 degree error in one tick.

    start_angles/target_angles only need to support the first three elements --
    a full gait-frame entry (`[roll, pitch, knee, is_swing]`) can be passed
    directly without the caller having to strip the swing flag first.
    """
    sx, sy, sz = ik_computer.calculate_fk(*start_angles[:3])
    tx, ty, tz = ik_computer.calculate_fk(*target_angles[:3])
    out = []
    for i in range(steps + 1):
        t = i / steps
        ik = ik_computer.calculate(x=sx + (tx - sx) * t,
                                   y=sy + (ty - sy) * t,
                                   z=sz + (tz - sz) * t)
        out.append([ik.roll, ik.pitch, ik.knee, 0.0])
    return out