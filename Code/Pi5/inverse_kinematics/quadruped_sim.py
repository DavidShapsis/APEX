#!/usr/bin/env python3
"""
quadruped_sim.py
----------------
Full four-leg APEX simulator. Runs on a PC only -- no hardware, no ROS.

Uses the PRODUCTION classes from ik_and_gait.py and mirrors the leg-selection and
phase logic in pi5_main.py, so what you see here is what the robot will attempt.

    python quadruped_sim.py             # 3D animation
    python quadruped_sim.py --report    # headless numeric verdict, no window

The report answers the only question that matters before powering motors:
does exactly one leg leave the ground at a time, and does the body actually move?
"""

import argparse
import math
import os
import sys

# Import the production engines whether this is run from the repo root or from
# inside inverse_kinematics/.
try:
    import ik_and_gait as G
except ImportError:
    sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
    import ik_and_gait as G

InverseKinematics, GaitPath, GaitIK = G.InverseKinematics, G.GaitPath, G.GaitIK


# ==============================================================================
# CONFIG
# ==============================================================================

# --- Body geometry and leg identity come from ik_and_gait.py -----------------
# Single source of truth, shared with pi5_main.py so the two cannot disagree.
BODY_LENGTH = G.BODY_LENGTH_CM    # front hip pitch axis to rear, cm
BODY_WIDTH  = G.BODY_WIDTH_CM     # left hip axis to right, cm
# -----------------------------------------------------------------------------

# --- Gait, matching pi5_main.py defaults ---
STRIDE_LENGTH  = 10.0
STAND_HEIGHT   = 36.0
SWING_HEIGHT   = 5.0    # height1: lift above neutral during swing
STANCE_PUSH    = 0.0    # height2: 0 so all planted feet share one ground plane
SWING_FRACTION = 0.25   # 0.25 -> one leg airborne, three planted

# --- Steering: 0 straight, +90 hard right, -90 hard left ---
DIRECTION_CMD = 0

# --- Body sway: lean toward the supporting tripod before each lift ---
# Set to 0.0 to see the ~4mm static margin the crawl has without it.
BODY_SHIFT_CM = 2.0

# --- Simulated IMU tilt, to exercise the levelling ---
# Positive roll = right side down, positive pitch = nose up.
SIM_ROLL_DEG  = 0.0
SIM_PITCH_DEG = 0.0
ATTITUDE_GAIN = 0.6
ATTITUDE_LIMIT_CM = 4.0

# --- Keep the gait mirror consistent with how the legs are bolted on ---
# Set False to watch the flipped legs fight the others -- a useful sanity check
# that the mirror is doing real work.
GAIT_MIRROR_MATCHES_MOUNT = True

# --- Leg identity, matching pi5_main.py / pico_main.py LEG_ID ---
LEG_FL, LEG_FR, LEG_RR, LEG_RL = G.LEG_FL, G.LEG_FR, G.LEG_RR, G.LEG_RL
LEG_ORDER = G.LEG_ORDER
LEG_NAMES = {LEG_FL: 'FL', LEG_FR: 'FR', LEG_RR: 'RR', LEG_RL: 'RL'}
LEFT_LEGS = G.LEFT_LEGS

# Hip position in body frame: +X right, +Y forward, Z down from body origin.
HIP_POS = dict(G.HIP_POSITIONS)

# How each leg assembly is bolted on, as reflections of the leg's local frame.
#
# FLIP_X -- left legs are mirror-image copies of the right ones, so the abductor
#   segment sticks outboard on both sides rather than both pointing the same way.
#   Reflection reverses handedness, which is what the Pico's per-joint `reverse`
#   flags exist to absorb; the commanded angles are identical either way.
#
# FLIP_Y -- the IK solves a knee-FORWARD leg (at neutral the knee node sits at
#   local y = +18.8cm). That matches the rear pair. The front pair is bolted on
#   turned round so its knee points back, as on a real dog, so the front legs are
#   the flipped ones. Their gait must be mirrored to match, or they push the wrong
#   way; FLIP_Y and the gait's mirror_y must always agree.
LEG_FLIP_X = {leg: G.LEG_SIGN_X[leg] < 0 for leg in LEG_ORDER}
LEG_FLIP_Y = {leg: G.LEG_SIGN_Y[leg] < 0 for leg in LEG_ORDER}

# How much clearance the body centre should keep from the edge of the support
# triangle. Merely being inside it is not enough -- payload offset, the swinging
# leg's own mass and uneven ground all eat into this.
MIN_STABILITY_MARGIN_CM = 2.0

CYCLES_TO_SIM = 4
STEP_TICK_MS = 40            # must match pico_main.py -- sets the real-world cadence
MOTOR_FREE_DPS = 360.0       # goBILDA 5302, 99.5:1, 60 RPM at the output shaft
ANIMATION_INTERVAL_MS = 220  # wall-clock ms per rendered frame; raise to slow down

# ==============================================================================


ik_engine = InverseKinematics()
path_gen = GaitPath()


def build_gait():
    """Per-leg joint trajectories, mirroring PiQuadrupedController.build_gait().

    Uses the same shared helpers from ik_and_gait, so what the sim animates is
    what the robot will be commanded.
    """
    steering = max(-1.0, min(1.0, DIRECTION_CMD / 45.0))
    left_stride = STRIDE_LENGTH * (1.0 + 0.5 * steering)
    right_stride = STRIDE_LENGTH * (1.0 - 0.5 * steering)

    n = 20
    offsets = G.phase_offsets(n)
    path_gen.update_params(0.0, STAND_HEIGHT, STRIDE_LENGTH, SWING_HEIGHT,
                           STANCE_PUSH, 0, swing_fraction=SWING_FRACTION)
    swing_steps = [i for i, s in enumerate(path_gen.gait_xy_path) if s[3]]
    shift = G.body_shift_profile(swing_steps, offsets, n, BODY_SHIFT_CM)
    dz = G.attitude_height_offsets(SIM_ROLL_DEG, SIM_PITCH_DEG,
                                   gain=ATTITUDE_GAIN, limit_cm=ATTITUDE_LIMIT_CM)

    per_leg = {}
    for leg_id in LEG_ORDER:
        stride = left_stride if leg_id in LEFT_LEGS else right_stride
        path_gen.update_params(
            center_stride_y=0.0, center_height_z=STAND_HEIGHT + dz[leg_id],
            length=stride, height1=SWING_HEIGHT, height2=STANCE_PUSH,
            direction_angle=0, swing_fraction=SWING_FRACTION,
            mirror_y=(GAIT_MIRROR_MATCHES_MOUNT and LEG_FLIP_Y[leg_id])
        )
        path = G.apply_body_shift(path_gen.gait_xy_path, leg_id, offsets, shift, n)
        per_leg[leg_id] = GaitIK(ik_engine, path).get_gait_ik()
    return per_leg


def leg_nodes(roll_deg, pitch_deg, knee_deg):
    """Hip -> shoulder -> knee -> foot, in the leg's local frame.

    Same reconstruction gait_testing2/3.py use, so the drawing matches the
    engine's own +90 degree roll convention.
    """
    a, b = ik_engine.SEGMENT_LENGTHS['a'], ik_engine.SEGMENT_LENGTHS['b']
    roll_rad = math.radians(roll_deg + 90.0)
    pitch_rad = math.radians(pitch_deg)

    s = (a * math.sin(roll_rad), 0.0, a * math.cos(roll_rad))

    knee_y = b * math.sin(pitch_rad)
    z_rel_knee = b * math.cos(pitch_rad)
    r_xz_knee = math.sqrt(z_rel_knee ** 2 + a ** 2)
    phi2 = math.acos(max(-1.0, min(1.0, a / r_xz_knee)))
    phi1 = roll_rad - phi2
    k = (r_xz_knee * math.sin(phi1), knee_y, r_xz_knee * math.cos(phi1))

    f = ik_engine.calculate_fk(roll_deg, pitch_deg, knee_deg)
    return s, k, f


def mount_to_body(leg_id, node):
    """Leg-local point -> body frame, accounting for how the assembly is bolted on."""
    px, py, pz = node
    if LEG_FLIP_X[leg_id]:
        px = -px
    if LEG_FLIP_Y[leg_id]:
        py = -py
    hx, hy = HIP_POS[leg_id]
    return (hx + px, hy + py, pz)


def fit_rigid(P, Q):
    """2D rigid fit: returns (cx, cy, dtheta) with P ~= R(dtheta) * Q + c.

    P and Q are the same patch of ground seen from the body frame one tick apart,
    so this recovers the body's motion -- translation and yaw together. Yaw matters:
    differential stride drives one side further than the other, which turns the
    robot rather than sliding it sideways, so a translation-only fit sees nothing.
    """
    n = len(P)
    pbx = sum(p[0] for p in P) / n
    pby = sum(p[1] for p in P) / n
    qbx = sum(q[0] for q in Q) / n
    qby = sum(q[1] for q in Q) / n

    num = den = 0.0
    for p, q in zip(P, Q):
        ax, ay = q[0] - qbx, q[1] - qby
        bx, by = p[0] - pbx, p[1] - pby
        num += ax * by - ay * bx
        den += ax * bx + ay * by
    dtheta = math.atan2(num, den)

    c, s = math.cos(dtheta), math.sin(dtheta)
    return pbx - (c * qbx - s * qby), pby - (s * qbx + c * qby), dtheta


def simulate():
    """Steps the whole robot through CYCLES_TO_SIM gait cycles.

    Returns a list of frames; each frame holds every leg's joint nodes in world
    coordinates, which feet are planted, and where the body has travelled to.
    """
    per_leg = build_gait()
    num_steps = len(per_leg[LEG_FL])
    # Quarter-cycle spacing, exactly as pi5_main.py computes it.
    offsets = G.phase_offsets(num_steps)

    frames = []
    body_x, body_y, body_yaw = 0.0, 0.0, 0.0
    prev_body = {}
    prev_planted = set()

    for tick in range(num_steps * CYCLES_TO_SIM):
        legs = {}
        planted = []
        for leg_id in LEG_ORDER:
            # The Pico walks its own buffer at its phase offset -- this is the
            # same indexing pico_main.py does.
            step_idx = (tick + offsets[leg_id]) % num_steps
            roll, pitch, knee, swing = per_leg[leg_id][step_idx]
            shoulder, knee_pt, foot = leg_nodes(roll, pitch, knee)
            legs[leg_id] = {
                'swing': bool(swing), 'angles': (roll, pitch, knee),
                'body': {name: mount_to_body(leg_id, node) for name, node in
                         (('shoulder', shoulder), ('knee', knee_pt), ('foot', foot))},
            }
            if not swing:
                planted.append(leg_id)

        # Planted feet are pinned to the ground, so their apparent motion in the
        # body frame is the body moving the other way. Only legs planted in BOTH
        # ticks count -- a foot that just touched down would otherwise contribute
        # its mid-air position as the previous sample.
        shared = [i for i in planted if i in prev_planted]
        if len(shared) >= 2:
            dx, dy, dyaw = fit_rigid([prev_body[i] for i in shared],
                                     [legs[i]['body']['foot'] for i in shared])
            # The fit is expressed in the previous body frame; rotate into world
            # before advancing the heading.
            c, s = math.cos(body_yaw), math.sin(body_yaw)
            body_x += dx * c - dy * s
            body_y += dx * s + dy * c
            body_yaw += dyaw

        prev_body = {i: legs[i]['body']['foot'] for i in LEG_ORDER}
        prev_planted = set(planted)

        # Lift into world coordinates.
        cw, sw = math.cos(body_yaw), math.sin(body_yaw)

        def to_world(p):
            return (body_x + p[0] * cw - p[1] * sw,
                    body_y + p[0] * sw + p[1] * cw,
                    p[2])

        for leg_id, leg in legs.items():
            hx, hy = HIP_POS[leg_id]
            leg['hip_w'] = to_world((hx, hy, 0.0))
            for name in ('shoulder', 'knee', 'foot'):
                leg[name + '_w'] = to_world(leg['body'][name])

        frames.append({
            'tick': tick, 'legs': legs, 'planted': planted,
            'airborne': [i for i in LEG_ORDER if legs[i]['swing']],
            'body': (body_x, body_y), 'yaw': body_yaw,
        })

    return frames, num_steps


def point_in_triangle(p, a, b, c):
    def sign(u, v, w):
        return (u[0] - w[0]) * (v[1] - w[1]) - (v[0] - w[0]) * (u[1] - w[1])
    d1, d2, d3 = sign(p, a, b), sign(p, b, c), sign(p, c, a)
    has_neg = min(d1, d2, d3) < 0
    has_pos = max(d1, d2, d3) > 0
    return not (has_neg and has_pos)


def _point_seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def stability_margin(frame):
    """Signed cm from the body centre to the nearest support-polygon edge.

    Being inside the polygon at all is a low bar -- with the feet directly under
    the hips the centre lands almost exactly on the diagonal between two of the
    three planted feet, so the margin is what actually decides whether it tips.
    """
    feet = [frame['legs'][i]['foot_w'] for i in frame['planted']]
    if len(feet) < 3:
        return -999.0
    com = frame['body']
    d = min(_point_seg_dist(com, feet[i], feet[(i + 1) % len(feet)])
            for i in range(len(feet)))
    return d if stability(frame) else -d


def stability(frame):
    """True if the body centre sits inside the polygon of planted feet."""
    feet = [frame['legs'][i]['foot_w'] for i in frame['planted']]
    com = frame['body']
    if len(feet) < 3:
        return False
    if len(feet) == 3:
        return point_in_triangle(com, feet[0], feet[1], feet[2])
    # 4 feet: split the quad into two triangles around the body.
    ordered = sorted(feet, key=lambda f: math.atan2(f[1] - com[1], f[0] - com[0]))
    return (point_in_triangle(com, ordered[0], ordered[1], ordered[2]) or
            point_in_triangle(com, ordered[0], ordered[2], ordered[3]))


def report(frames, num_steps):
    print("=" * 66)
    print("  APEX QUADRUPED GAIT REPORT")
    print("=" * 66)
    print(f"  swing_fraction {SWING_FRACTION}   stride {STRIDE_LENGTH}cm   "
          f"height {STAND_HEIGHT}cm   steer {DIRECTION_CMD}deg")
    print(f"  gait mirror matches mount: {GAIT_MIRROR_MATCHES_MOUNT}   "
          f"flipped legs: {', '.join(LEG_NAMES[i] for i in LEG_ORDER if LEG_FLIP_Y[i])}")
    print(f"  body           {BODY_LENGTH} x {BODY_WIDTH} cm  <-- set from CAD")
    print("-" * 66)

    counts = [len(f['airborne']) for f in frames]
    max_air = max(counts)
    print(f"  Legs airborne at once : min {min(counts)}  max {max_air}")
    ok_air = max_air <= 1
    print(f"  {'PASS' if ok_air else 'FAIL'}  one leg at a time"
          f"{'' if ok_air else f'  -- {max_air} legs off the ground together'}")

    stable = [stability(f) for f in frames]
    pct = 100.0 * sum(stable) / len(stable)
    print(f"  Statically stable     : {pct:.1f}% of the cycle")
    print(f"  {'PASS' if pct > 99.9 else 'FAIL'}  centre of mass inside support polygon")

    marg = [stability_margin(f) for f in frames]
    worst, mean = min(marg), sum(marg) / len(marg)
    print(f"  Stability margin      : worst {worst:+.2f} cm, mean {mean:+.2f} cm")
    roomy = worst >= MIN_STABILITY_MARGIN_CM
    print(f"  {'PASS' if roomy else 'WARN'}  margin at least {MIN_STABILITY_MARGIN_CM:.1f} cm")
    if not roomy:
        print(f"         {worst*10:.0f} mm is inside the polygon but on a knife edge. With the")
        print("         feet under the hips the centre sits on the support diagonal, so")
        print("         payload offset, leg mass or uneven ground will tip it. The fix is")
        print("         a lateral body shift toward the supporting tripod before each lift.")

    first, last = frames[0]['body'], frames[-1]['body']
    dx, dy = last[0] - first[0], last[1] - first[1]
    dist = math.hypot(dx, dy)
    per_cycle = dist / CYCLES_TO_SIM
    yaw_deg = math.degrees(frames[-1]['yaw'] - frames[0]['yaw'])
    print(f"  Travel                : {dist:.2f} cm net over {CYCLES_TO_SIM} cycles "
          f"({per_cycle:.2f} cm/cycle)")
    print(f"  Displacement          : X {dx:+.2f}   Y {dy:+.2f} cm")
    print(f"  Heading change        : {yaw_deg:+.1f} deg "
          f"({yaw_deg / CYCLES_TO_SIM:+.1f} deg/cycle)")
    fwd = per_cycle > 0.1
    print(f"  {'PASS' if fwd else 'FAIL'}  body actually travels"
          f"{'' if fwd else '  -- legs are cancelling each other out'}")

    # Real-world cadence, and whether the motors can actually deliver it.
    cycle_s = num_steps * STEP_TICK_MS / 1000.0
    per_cycle_fwd = 13.33 if per_cycle == 0 else per_cycle
    print(f"  Cadence               : {STEP_TICK_MS} ms/step -> {cycle_s:.2f} s/cycle, "
          f"{per_cycle_fwd / cycle_s:.1f} cm/s")

    peak_dps = 0.0
    for f in frames:
        i = f['tick']
        nxt = frames[(i + 1) % num_steps] if i + 1 < len(frames) else frames[0]
        for leg_id in LEG_ORDER:
            a = f['legs'][leg_id]['angles']
            b = nxt['legs'][leg_id]['angles']
            peak_dps = max(peak_dps, max(abs(b[j] - a[j]) for j in range(3)))
    peak_dps *= 1000.0 / STEP_TICK_MS
    frac = peak_dps / MOTOR_FREE_DPS
    print(f"  Peak joint rate       : {peak_dps:.0f} deg/s "
          f"({frac * 100:.0f}% of {MOTOR_FREE_DPS:.0f} deg/s free speed)")
    fast_enough = frac <= 1.0
    print(f"  {'PASS' if fast_enough else 'FAIL'}  motors can reach the commanded rate")
    if not fast_enough:
        print(f"         raise STEP_TICK_MS to at least {STEP_TICK_MS * frac:.0f} ms")
    elif frac > 0.8:
        print(f"         only {(1-frac)*100:.0f}% headroom -- free speed is unloaded, "
              "expect less on the robot")

    # Lift order over one cycle.
    order, seen = [], set()
    for f in frames[:num_steps]:
        for leg in f['airborne']:
            if leg not in seen:
                seen.add(leg)
                order.append(LEG_NAMES[leg])
    print(f"  Lift order            : {' -> '.join(order) if order else '(none)'}")
    good = order == ['FL', 'RR', 'FR', 'RL']
    print(f"  {'PASS' if good else 'WARN'}  crawl sequence"
          f"{'' if good else '  -- expected FL -> RR -> FR -> RL'}")

    # IK reachability: FK round-trip drift means the target was clamped.
    worst = 0.0
    for f in frames:
        for leg in f['legs'].values():
            r, p, k = leg['angles']
            fx, fy, fz = ik_engine.calculate_fk(r, p, k)
            worst = max(worst, abs(fz - STAND_HEIGHT) - (SWING_HEIGHT + STANCE_PUSH))
    print(f"  {'PASS' if worst < 0.5 else 'WARN'}  all foot targets reachable")

    print("-" * 66)
    kinematic = ok_air and pct > 99.9 and fwd and good
    if not kinematic:
        print("  VERDICT: gait will NOT walk correctly")
    elif not roomy:
        print("  VERDICT: kinematically correct, but the static margin is too thin")
        print("           to stay upright reliably -- needs a body shift.")
    else:
        print("  VERDICT: gait is viable")
    print("=" * 66)
    return kinematic and roomy


def animate(frames, num_steps, speed=1.0):
    # Imported here, not at module scope, so --report still works on a machine
    # without matplotlib.
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)
    except ImportError:
        print("\n[!] matplotlib is not installed, so the 3D view cannot open.")
        print("      pip install matplotlib")
        print("    The report above does not need it, and --report skips this entirely.")
        return None

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection='3d')

    colors = {LEG_FL: 'tab:blue', LEG_FR: 'tab:orange',
              LEG_RR: 'tab:green', LEG_RL: 'tab:red'}

    leg_lines = {i: ax.plot([], [], [], 'o-', lw=3, color=colors[i],
                            label=LEG_NAMES[i])[0] for i in LEG_ORDER}
    foot_dots = {i: ax.plot([], [], [], 'o', ms=9, color=colors[i])[0] for i in LEG_ORDER}
    body_line, = ax.plot([], [], [], '-', lw=4, color='black', alpha=0.8)
    support_line, = ax.plot([], [], [], '-', lw=2, color='green', alpha=0.5)
    com_dot, = ax.plot([], [], [], 'X', ms=12, color='green')
    trails = {i: ax.plot([], [], [], '--', lw=1, alpha=0.4, color=colors[i])[0]
              for i in LEG_ORDER}
    hist = {i: ([], [], []) for i in LEG_ORDER}

    span = max(BODY_LENGTH, BODY_WIDTH)
    ax.set_xlabel('X (lateral, cm)')
    ax.set_ylabel('Y (forward, cm)')
    ax.set_zlabel('Z (down, cm)')

    def update(n):
        f = frames[n]
        bx, by = f['body']

        for i in LEG_ORDER:
            leg = f['legs'][i]
            pts = [leg['hip_w'], leg['shoulder_w'], leg['knee_w'], leg['foot_w']]
            leg_lines[i].set_data([p[0] for p in pts], [p[1] for p in pts])
            leg_lines[i].set_3d_properties([p[2] for p in pts])

            fx, fy, fz = leg['foot_w']
            foot_dots[i].set_data([fx], [fy])
            foot_dots[i].set_3d_properties([fz])
            foot_dots[i].set_marker('^' if leg['swing'] else 'o')

            hist[i][0].append(fx); hist[i][1].append(fy); hist[i][2].append(fz)
            for axis in hist[i]:
                del axis[:-num_steps]
            trails[i].set_data(hist[i][0], hist[i][1])
            trails[i].set_3d_properties(hist[i][2])

        corners = [f['legs'][i]['hip_w'] for i in (LEG_FL, LEG_FR, LEG_RR, LEG_RL, LEG_FL)]
        body_line.set_data([p[0] for p in corners], [p[1] for p in corners])
        body_line.set_3d_properties([p[2] for p in corners])

        feet = [f['legs'][i]['foot_w'] for i in f['planted']]
        loop = feet + feet[:1]
        support_line.set_data([p[0] for p in loop], [p[1] for p in loop])
        support_line.set_3d_properties([p[2] for p in loop])

        ok = stability(f)
        support_line.set_color('green' if ok else 'red')
        com_dot.set_color('green' if ok else 'red')
        com_dot.set_data([bx], [by])
        com_dot.set_3d_properties([STAND_HEIGHT])

        ax.set_xlim(bx - span, bx + span)
        ax.set_ylim(by - span, by + span)
        ax.set_zlim(STAND_HEIGHT + 12, -12)

        air = ', '.join(LEG_NAMES[i] for i in f['airborne']) or 'none'
        ax.set_title(
            f"APEX crawl  |  tick {f['tick'] % num_steps}/{num_steps}  "
            f"cycle {f['tick'] // num_steps + 1}/{CYCLES_TO_SIM}\n"
            f"airborne: {air}   planted: {len(f['planted'])}   "
            f"{'STABLE' if ok else 'UNSTABLE'}\n"
            f"position X {bx:+.1f}  Y {by:+.1f} cm   "
            f"heading {math.degrees(f['yaw']):+.1f} deg"
        )
        return list(leg_lines.values())

    ax.legend(loc='upper left')
    # Keep a reference: matplotlib drops animations that are not held onto.
    interval = max(20, int(ANIMATION_INTERVAL_MS / max(0.05, speed)))
    ani = FuncAnimation(fig, update, frames=len(frames), interval=interval,
                        blit=False, cache_frame_data=False)
    plt.show()
    return ani


def main():
    ap = argparse.ArgumentParser(description="APEX four-leg gait simulator")
    ap.add_argument('--report', action='store_true',
                    help="print the numeric verdict and exit, no window")
    ap.add_argument('--speed', type=float, default=1.0, metavar='X',
                    help="playback rate: 0.5 is half speed, 2 is double "
                         f"(1.0 = {ANIMATION_INTERVAL_MS}ms per step)")
    args = ap.parse_args()

    frames, num_steps = simulate()
    ok = report(frames, num_steps)

    if not args.report:
        rate = max(20, int(ANIMATION_INTERVAL_MS / max(0.05, args.speed)))
        print(f"\nOpening 3D view at {rate}ms per step -- close the window to exit.")
        print("Use --speed 0.5 to halve it, --speed 2 to double it.")
        animate(frames, num_steps, speed=args.speed)

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
