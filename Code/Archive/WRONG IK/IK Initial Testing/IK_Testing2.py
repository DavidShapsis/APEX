import math

# --- Configuration ---
LINKS = {'a': 5, 'b': 10.0, 'c': 10.0}
TARGET = {'x': 2.5, 'y': 0, 'z': -15.0} # X=Side, Y=Forward, Z=Up (Negative is down)

def get_leg_angles(x, y, z, links):
    a, b, c = links['a'], links['b'], links['c']

    # 1. Hip Roll (Rotation around Y axis, driven by X and Z)
    r_xz = math.sqrt(x**2 + z**2)
    # atan2(x, -z) assumes body is at Z=0 and ground is negative Z
    hip_roll = math.atan2(x, -z) + math.acos(clip(a / r_xz))

    # 2. Projection into the Pitch/Knee plane
    z_rel = math.sqrt(max(0, r_xz**2 - a**2))
    y_rel = y
    dist_to_foot_sq = y_rel**2 + z_rel**2
    dist_to_foot = math.sqrt(dist_to_foot_sq)

    # 3. Hip Pitch
    alpha = math.atan2(y_rel, z_rel)
    beta = math.acos(clip((b**2 + dist_to_foot_sq - c**2) / (2 * b * dist_to_foot)))
    hip_pitch = alpha + beta

    # 4. Knee Angle
    knee_angle = math.acos(clip((b**2 + c**2 - dist_to_foot_sq) / (2 * b * c)))

    return {
        "roll": math.degrees(hip_roll),
        "pitch": math.degrees(hip_pitch),
        "knee": math.degrees(knee_angle)
    }

def clip(val):
    return max(-1.0, min(1.0, val))

# --- Execution ---
try:
    angles = get_leg_angles(TARGET['x'], TARGET['y'], TARGET['z'], LINKS)
    print(f"--- Joint Angles (X=Side, Y=Fwd, Z=Up) ---")
    print(f"Roll:  {angles['roll']:.2f}°")
    print(f"Pitch: {angles['pitch']:.2f}°")
    print(f"Knee:  {angles['knee']:.2f}°")
except ValueError:
    print("Error: Coordinate out of reach.")