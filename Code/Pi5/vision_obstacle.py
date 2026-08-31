"""
vision_obstacle.py
------------------
Monocular-depth obstacle detection and avoidance steering.

No ROS, no serial, no IK imports -- this module only ever sees camera frames and
returns a steering suggestion. `pi5_main.py` decides whether to act on it. That
keeps it runnable standalone (see the __main__ block at the bottom) and means a
failure in here degrades to "no avoidance", never to a robot that cannot walk.

Pipeline
    frame -> depth map -> per-bin obstacle costmap -> committed steering decision

The depth model is Depth-Anything-V2-Small in ONNX form, pretrained -- nothing
here is trained. It emits *relative* inverse depth (bright = near), not metres,
which is why every threshold below is a normalised 0..1 value tuned by eye
rather than a distance.

Why a state machine and not a per-frame reaction
    The GPS navigator recomputes the bearing to the waypoint from the robot's
    current position on every control-loop pass, so it is already self-correcting
    -- a detour does not need an explicit "rejoin the track" manoeuvre, it just
    needs avoidance to stop overriding once the obstacle is behind. What it does
    need is protection against the obvious failure mode: steer away, obstacle
    leaves the field of view, nav points straight back at it, repeat forever.
    AvoidancePlanner commits to one side for the duration of a detour and holds
    that heading until the body has physically cleared the obstacle.
"""

import math
import os
import threading
import time

import numpy as np

try:
    import cv2
except ImportError:                                     # pragma: no cover
    cv2 = None

try:
    import onnxruntime as ort
except ImportError:                                     # pragma: no cover
    ort = None


_HERE = os.path.dirname(os.path.abspath(__file__))

# Checked in order. The second/third entries are where the test notebook
# (vision_test/obstacle_avoidance_test.ipynb) downloads the model, so a model
# already fetched for bench testing is picked up without being moved.
DEFAULT_MODEL_PATHS = (
    os.path.join(_HERE, "models", "depth_anything_v2_s.onnx"),
    os.path.join(_HERE, "vision_test", "models", "depth_anything_v2_s.onnx"),
    os.path.join(_HERE, "vision_test", "models", "depth_anything_v2_s_int8.onnx"),
)

MODEL_URL = ("https://huggingface.co/onnx-community/depth-anything-v2-small"
             "/resolve/main/onnx/model.onnx")

# Inference resolution. MUST be a multiple of 14 (the model's patch size).
# 266 runs at roughly 1-3 fps on the Pi 5 CPU, which is ample: the crawl gait
# moves one leg at a time at ~16.7 cm/s, so the scene barely changes between
# frames. 518 is native and sharper but several times slower.
INPUT_SIZE = 266

# ImageNet normalisation, from the model's own preprocessor_config.json.
_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
_STD = np.array([0.229, 0.224, 0.225], np.float32)

# --- Costmap geometry -------------------------------------------------------
# Band of the frame treated as "the ground ahead". Excludes the sky (which
# always reads far and would dilute the bins) and the very bottom edge (which
# is the robot's own nose). VERIFY THIS ON THE BENCH against a still from the
# real camera mount -- it should cover roughly 0.5-2 m in front of the feet.
ROI_TOP = 0.45
ROI_BOTTOM = 0.95

N_BINS = 9              # vertical slices across the field of view
BLOCK_FRAC = 0.25       # fraction of a bin flagged before it counts as blocked

# --- Ground-relative detection ---------------------------------------------
# An absolute depth threshold does NOT work here, and it is worth being explicit
# about why. For a forward-facing camera on flat ground, distance to the ground
# at image row r goes as 1/(r - horizon), so the inverse depth the model emits
# ramps smoothly from far at the top of the frame to near at the bottom. The
# bottom of the ROI is therefore ALWAYS "near" -- it is the ground half a metre
# in front of the feet. A single threshold across the band flags that ground as
# an obstacle: measured on a synthetic scene matching a real capture, bare open
# ground scored 0.42 in every bin against a 0.25 block threshold, i.e. "fully
# blocked" with nothing in front of the robot at all.
#
# What actually marks an obstacle is being nearer than the ground SHOULD be at
# that row. An upright object occupies rows that would otherwise show ground
# much further away, so it stands out sharply against its own row.
#
# A useful side effect: because the reference is drawn from the row itself, a
# uniform slope shifts the whole row together and is NOT flagged. Only localised
# protrusions are, which is the desired behaviour on terrain.

# Percentile of each row used as its ground reference. Deliberately below the
# median, biasing toward the far side, so a wide object cannot drag the
# reference up to its own depth and hide itself -- this still finds the ground
# while an obstacle covers up to ~70% of a row.
GROUND_REF_PCT = 30.0

# How much nearer than its row's reference a pixel must read to count as an
# obstacle, in normalised depth. Measured insensitive between 0.05 and 0.25 on
# the test scenes (all isolated a centred figure identically), so this is a
# comfortable middle rather than a knife edge.
OBSTACLE_MARGIN = 0.12

# Fallback for the one case the relative test cannot see: something filling a
# column top to bottom, leaving no nearer-than-reference contrast anywhere in
# it. On open ground a column ramps strongly from far at the top to near at the
# bottom; a wall flattens that ramp. A bin whose own vertical gradient falls
# below this is treated as fully blocked. Raising it is the safe direction (more
# halting, fewer missed walls); it needs a bench check against the real mount,
# since a more steeply downward-angled camera sees less depth range in the band.
FLAT_GRADIENT_MIN = 0.10

# Horizontal field of view of the USB camera, degrees. Only used to convert a
# bin index into a steering angle, so an error here scales the turn command
# rather than breaking detection. Measure it if turns come out consistently
# too shallow or too sharp.
FOV_DEG = 60.0

# Widest part of the robot, cm. BODY_WIDTH_CM in ik_and_gait.py is 53, but that
# is the HIP PIVOT spacing -- the abductor segment (a = 9.65) sticks outboard on
# both sides, so what actually has to fit through a gap is wider than the hips.
ROBOT_WIDTH_CM = 53.0 + 2 * 9.65      # ~72 cm

# How many adjacent bins must be clear for the body to fit through.
#
# The honest version of this is range-dependent and the costmap cannot express
# it: 72 cm subtends 71.7 deg at 0.5 m but only 20.5 deg at 2 m, i.e. anywhere
# from 10.8 bins down to 3.1, and the ROI band mixes all those ranges into one
# flat map with no depth axis. So this is a fixed compromise, not a derivation.
#
# 3 here plus INFLATE_BINS=1 on each side means 5 raw bins must be clear:
# 33.3 deg, which covers the true body width beyond ~1.2 m. Closer in it
# under-provisions on paper -- but an obstacle that close also fills enough bins
# to trip BLOCKED, which halts rather than squeezing through. Raising this to 5
# would demand 46.7 deg of a 60 deg view and leave almost nothing passable
# (INFLATE_BINS has the measured note on that failure mode).
CORRIDOR_BINS = 3

# Obstacle inflation, in bins on each side (the configuration-space trick).
# Kept deliberately small. 60 deg of view across 9 bins is only 6.7 deg per bin,
# so inflating by 2 knocks out 5 of the 9 and leaves nothing passable at all --
# measured: the planner went straight to BLOCKED for an obstacle it could
# comfortably have walked around. One bin trims the "clear" heading back off the
# obstacle's edge without closing every gap.
INFLATE_BINS = 1

# Extra turn added beyond the edge of the gap, degrees, away from the obstacle.
# This -- not inflation -- is what buys lateral clearance, and it is the single
# most important number here.
#
# The planner picks the passable heading nearest the goal, which is by
# construction the one that *just* misses. Committing to exactly that heading
# means skimming the obstacle, and at this robot's speeds there is no room for
# that: detection only reaches ~2 m, and at the avoidance stride the turn rate
# is a few degrees per second, so a shallow command runs out of distance before
# it has produced any real sideways offset. Simulated against the real
# Navigator, no margin grazed every obstacle on the route; 15 deg cleared them.
#
# This also carries the half-body-width the fixed corridor cannot: 15 deg at
# 1.5 m is ~39 cm of extra offset, against a 36 cm half-width.
AVOID_MARGIN_DEG = 15.0

# --- Planner timing ---------------------------------------------------------
# Minimum time committed to a side before the other side may be reconsidered.
# Without this the robot ping-pongs between two equally-good gaps.
COMMIT_MIN_S = 2.0

# How long to keep driving straight after the obstacle has left the field of
# view entirely, before steering is handed back to navigation.
#
# This is the parameter that decides whether the robot actually clears an
# obstacle or merely grazes it, and it is worth understanding before changing.
# The camera is at the front of a ~95 cm body, and the obstacle is still ~1 m
# ahead when it slides out of a 60 deg view -- so at the instant the frame goes
# clear, the robot has not passed anything yet. Hand steering straight back to
# the navigator here and it cuts the corner into what it just avoided.
#
# A distance would be the natural unit; there is no odometry, so time at the
# known avoidance stride (~10 cm/s) stands in. Swept in simulation against the
# real Navigator: 4 s gave +0.01 m of clearance (i.e. a graze), 10 s gave
# +0.02 m, and 20 s gave +0.50 m, after which it flattens -- 20 s buys the ~2 m
# needed to put the obstacle genuinely behind the rear legs. The cost is about
# 3% on route time. Lower it only if detours are visibly wider than they need
# to be, and re-measure clearance when you do.
CLEAR_HOLD_S = 20.0

# Time spent halted-in-place re-observing before trying an escape arc.
BLOCKED_HOLD_S = 2.0
# Ceiling on one escape arc before giving up and halting again.
ESCAPE_MAX_S = 6.0

# Stride multipliers. 0.0 means length=0 in the gait, which makes the feet lift
# and set back down in the same spot -- the robot marches in place, still
# standing and still IMU-levelled. This is NOT the dashboard STOP, which cuts
# holding torque and would let it sag.
AVOID_STRIDE_SCALE = 0.6
ESCAPE_STRIDE_SCALE = 0.35
HALT_STRIDE_SCALE = 0.0

# A costmap older than this is not trusted. At ~2 fps that is several missed
# frames, which means the camera or the worker has stopped.
STALE_AFTER_S = 3.0

# pi5_main clamps the steer command to +/-90 deg (beyond which nothing changes:
# 90 already means "spin in place"). ESCAPE asks for the full 90 to rotate as
# fast as it can; a committed detour is trimmed off the gap edge and rarely
# reaches half this.
MAX_STEER_DEG = 90.0


class AvoidState:
    """Where the planner is in a detour. Deliberately NOT part of RobotState --
    that enum already conflates operating mode with transient activity and is
    flagged in KNOWN_ISSUES; adding a fourth meaning would make it worse."""
    OFF = "OFF"             # disabled, or no trustworthy costmap
    CLEAR = "CLEAR"         # corridor open, navigation steers
    AVOIDING = "AVOIDING"   # obstacle ahead, committed to a side
    CLEARING = "CLEARING"   # corridor reopened, driving past before turning back
    BLOCKED = "BLOCKED"     # no gap anywhere, halted in place
    ESCAPE = "ESCAPE"       # slow committed arc, hunting for a way out

    # Small integer codes, for the dashboard status array.
    CODES = {OFF: 0, CLEAR: 1, AVOIDING: 2, CLEARING: 3, BLOCKED: 4, ESCAPE: 5}


class Decision:
    """What the planner wants the robot to do this tick.

    steer_deg     -- in the same convention as /apex/navigation/cmd_dir:
                     negative left, positive right, saturating at
                     +/-MAX_STEER_DEG (90, which pi5_main reads as a spin).
    stride_scale  -- multiplier on the base stride length. 0.0 marches in place.
    """

    __slots__ = ("steer_deg", "stride_scale", "state", "reason", "fresh", "ms")

    def __init__(self, steer_deg, stride_scale, state, reason, fresh=True, ms=0.0):
        self.steer_deg = float(steer_deg)
        self.stride_scale = float(stride_scale)
        self.state = state
        self.reason = reason
        self.fresh = fresh
        # Wall-clock cost of producing this decision, filled in by whoever ran
        # the model. plan() leaves it at 0 -- it does not run inference.
        self.ms = float(ms)

    @property
    def overriding(self):
        """True when the planner is actively changing what the robot would
        otherwise have done."""
        return self.state in (AvoidState.AVOIDING, AvoidState.CLEARING,
                              AvoidState.BLOCKED, AvoidState.ESCAPE)

    def __repr__(self):
        return (f"<Decision {self.state} steer={self.steer_deg:+.0f} "
                f"stride={self.stride_scale:.2f} ({self.reason})>")


# =============================================================================
# Depth
# =============================================================================

class DepthEstimator:
    """Wraps the ONNX depth model. Never raises on a bad frame -- returns None."""

    def __init__(self, model_path, input_size=INPUT_SIZE, threads=None):
        if ort is None:
            raise RuntimeError("onnxruntime is not installed")
        if cv2 is None:
            raise RuntimeError("opencv-python is not installed")

        opts = ort.SessionOptions()
        # Leave cores for the 100 Hz control loop and the four serial writers.
        # ONNX Runtime releases the GIL during inference, so the worker thread
        # competes for CPU but does not block Python execution elsewhere.
        opts.intra_op_num_threads = threads or max(1, (os.cpu_count() or 4) - 2)
        self.session = ort.InferenceSession(str(model_path), opts,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.input_size = input_size

    def _preprocess(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.input_size, self.input_size),
                         interpolation=cv2.INTER_CUBIC)
        x = (rgb.astype(np.float32) / 255.0 - _MEAN) / _STD
        return np.transpose(x, (2, 0, 1))[None].astype(np.float32)

    def infer(self, bgr):
        """HxW float32 in 0..1 where 1.0 is nearest, or None on failure."""
        try:
            h, w = bgr.shape[:2]
            raw = self.session.run([self.output_name],
                                   {self.input_name: self._preprocess(bgr)})[0]
            depth = cv2.resize(np.squeeze(raw).astype(np.float32), (w, h),
                               interpolation=cv2.INTER_CUBIC)
            depth -= depth.min()
            peak = depth.max()
            if peak > 1e-6:
                depth /= peak
            # Sanity flip. The model's sign convention is fixed, but a bad
            # export or a future model swap would silently invert every
            # decision, so cross-check against the fact that the bottom of the
            # frame is always nearer than the top on a ground-facing camera.
            if depth[int(h * 0.85):].mean() < depth[:int(h * 0.15)].mean():
                depth = 1.0 - depth
            return depth
        except Exception:
            return None


def costmap_from_depth(depth, n_bins=N_BINS, roi_top=ROI_TOP,
                       roi_bottom=ROI_BOTTOM, margin=OBSTACLE_MARGIN,
                       ref_pct=GROUND_REF_PCT, flat_grad=FLAT_GRADIENT_MIN):
    """Depth map -> per-bin obstacle cost in 0..1, ordered left to right.

    Cost is the fraction of a bin reading nearer than the ground does at the
    same image row -- see the GROUND_REF_PCT / OBSTACLE_MARGIN notes above for
    why this is row-relative rather than an absolute depth threshold.
    """
    h = depth.shape[0]
    roi = depth[int(h * roi_top):int(h * roi_bottom), :]
    if roi.size == 0:
        return np.zeros(n_bins, np.float32)

    reference = np.percentile(roi, ref_pct, axis=1, keepdims=True)
    obstacle = (roi - reference) > margin
    cost = np.array([col.mean() for col in np.array_split(obstacle, n_bins, axis=1)],
                    np.float32)

    # Flat-gradient fallback, per bin rather than per frame, so a wall across
    # half the view blocks only that half.
    quarter = max(1, roi.shape[0] // 4)
    for i, col in enumerate(np.array_split(roi, n_bins, axis=1)):
        if float(col[-quarter:].mean() - col[:quarter].mean()) < flat_grad:
            cost[i] = 1.0
    return cost


# =============================================================================
# Planning
# =============================================================================

class AvoidancePlanner:
    """Turns a costmap plus a desired heading into a committed steering command.

    Pure and side-effect free apart from its own state, so it can be unit tested
    against hand-built costmaps without a camera or a model -- see __main__.
    """

    def __init__(self, n_bins=N_BINS, fov_deg=FOV_DEG, corridor_bins=CORRIDOR_BINS,
                 block_frac=BLOCK_FRAC, inflate_bins=INFLATE_BINS,
                 clear_hold_s=CLEAR_HOLD_S, commit_min_s=COMMIT_MIN_S,
                 blocked_hold_s=BLOCKED_HOLD_S, escape_max_s=ESCAPE_MAX_S,
                 avoid_margin_deg=AVOID_MARGIN_DEG):
        self.n_bins = n_bins
        self.block_frac = block_frac
        self.corridor_half = max(0, corridor_bins // 2)
        self.inflate_bins = max(0, inflate_bins)
        # Bin CENTRES, matching how costmap_from_depth actually slices the
        # image: array_split makes n equal columns, so column i covers
        # [-fov/2 + w*i, -fov/2 + w*(i+1)] and its centre is half a width in.
        # linspace(-fov/2, fov/2, n) would put the outermost centres at the very
        # edge of the view and space them fov/(n-1) apart, overstating every
        # bin's angle by up to half a bin.
        bin_w = fov_deg / n_bins
        self.bin_angles = -fov_deg / 2.0 + bin_w * (np.arange(n_bins) + 0.5)

        # Timings are per-instance rather than module globals so they can be
        # swept in simulation and tuned per robot without editing the module.
        self.clear_hold_s = clear_hold_s
        self.commit_min_s = commit_min_s
        self.blocked_hold_s = blocked_hold_s
        self.escape_max_s = escape_max_s
        self.avoid_margin_deg = avoid_margin_deg

        self.state = AvoidState.CLEAR
        self.state_since = time.monotonic()
        self.commit_sign = 0        # -1 left, +1 right, 0 uncommitted
        self.commit_deg = 0.0
        self.escape_deg = 0.0
        # When the whole field of view last became clear. Distinct from
        # state_since: the release timer must start when the obstacle actually
        # leaves the frame, not when the centre corridor first opened up.
        self.all_clear_since = None

    # -- helpers ------------------------------------------------------------

    def _enter(self, state, now):
        if state != self.state:
            self.state = state
            self.state_since = now

    def blocked_mask(self, cost):
        """Blocked bins, widened by INFLATE_BINS on each side.

        Everything that decides where the robot may drive works off this rather
        than the raw costmap, so the safety margin is applied in exactly one
        place and cannot be forgotten at a call site.
        """
        raw = cost >= self.block_frac
        if self.inflate_bins == 0:
            return raw
        mask = raw.copy()
        for shift in range(1, self.inflate_bins + 1):
            mask[shift:] |= raw[:-shift]        # spread left-to-right
            mask[:-shift] |= raw[shift:]        # and right-to-left
        return mask

    def _corridor_clear(self, mask, i):
        """Is a body-width corridor centred on bin i passable?

        Bins whose corridor runs off the edge of the frame are rejected rather
        than truncated -- clearance that was never observed is not clearance.
        """
        lo, hi = i - self.corridor_half, i + self.corridor_half
        if lo < 0 or hi >= self.n_bins:
            return False
        return not bool(np.any(mask[lo:hi + 1]))

    def _path_clear(self, mask):
        return self._corridor_clear(mask, self.n_bins // 2)

    def _pick(self, mask, desired_deg, side=0):
        """Index of the passable heading closest to where we actually want to
        go. `side` restricts the search to one hand (plus straight ahead) once
        a detour is committed. Returns None if nothing fits.

        Biasing toward `desired_deg` is what makes this "go around but keep
        aiming at the waypoint" rather than "go around": the moment the goal
        side opens up, the pick slides back toward it on its own.
        """
        cands = [i for i in range(self.n_bins) if self._corridor_clear(mask, i)]
        if side > 0:
            cands = [i for i in cands if self.bin_angles[i] >= 0]
        elif side < 0:
            cands = [i for i in cands if self.bin_angles[i] <= 0]
        if not cands:
            return None
        return min(cands, key=lambda i: abs(self.bin_angles[i] - desired_deg))

    def _steer_for(self, idx, sign=None):
        """Bin index -> steering command, pushed AVOID_MARGIN_DEG further away
        from the obstacle so the robot passes it rather than skims it."""
        edge = float(self.bin_angles[idx])
        if sign is None:
            sign = 1.0 if edge >= 0 else -1.0
        return float(np.clip(edge + self.avoid_margin_deg * sign,
                             -MAX_STEER_DEG, MAX_STEER_DEG))

    def _commit(self, idx, now):
        self.commit_sign = 1 if self.bin_angles[idx] >= 0 else -1
        self.commit_deg = self._steer_for(idx, self.commit_sign)
        self._enter(AvoidState.AVOIDING, now)

    def _choose_escape_side(self, cost):
        """Least-obstructed hand, for the last-resort arc."""
        mid = self.n_bins // 2
        left, right = cost[:mid].mean(), cost[mid + 1:].mean()
        self.escape_deg = -MAX_STEER_DEG if left <= right else MAX_STEER_DEG

    def reset(self):
        self.state = AvoidState.CLEAR
        self.state_since = time.monotonic()
        self.commit_sign = 0
        self.commit_deg = 0.0
        self.all_clear_since = None

    # -- main entry point ---------------------------------------------------

    def update(self, cost, desired_deg, now=None, fresh=True):
        now = time.monotonic() if now is None else now

        if cost is None or not fresh:
            # Fail open, not closed. A dead camera should leave the robot
            # walking exactly as it does with avoidance switched off, not halt
            # it in a field -- the FSR/ABORTED recovery path is still the
            # backstop for actually hitting something.
            self.reset()
            return Decision(desired_deg, 1.0, AvoidState.OFF,
                            "no fresh costmap", fresh=False)

        mask = self.blocked_mask(cost)
        path_clear = self._path_clear(mask)
        # Deliberately the RAW costmap, not the inflated mask: this asks "is
        # anything actually still visible", and inflating it would let a single
        # obstacle at the frame edge keep the detour alive indefinitely.
        #
        # An obstacle that has only just left the centre corridor is ~15 deg off
        # axis, which at 2 m is barely half a metre of lateral gap -- driving
        # straight at that point still clips it. The detour is not over until
        # nothing is in view at all.
        view_clear = not bool(np.any(cost >= self.block_frac))
        elapsed = now - self.state_since

        # --- transitions ---------------------------------------------------
        if self.state in (AvoidState.CLEAR, AvoidState.OFF):
            if self.state == AvoidState.OFF:
                self._enter(AvoidState.CLEAR, now)
            if not path_clear:
                idx = self._pick(mask, desired_deg)
                if idx is None:
                    self._choose_escape_side(cost)
                    self._enter(AvoidState.BLOCKED, now)
                else:
                    self._commit(idx, now)

        elif self.state == AvoidState.AVOIDING:
            if path_clear:
                self._enter(AvoidState.CLEARING, now)
            else:
                idx = self._pick(mask, desired_deg, side=self.commit_sign)
                if idx is not None:
                    # Refresh the target within the committed hand. This is the
                    # goal-seeking part: it tracks back toward desired_deg as
                    # soon as the geometry allows.
                    self.commit_deg = self._steer_for(idx, self.commit_sign)
                elif elapsed >= self.commit_min_s:
                    # Committed side has closed off and we have held it long
                    # enough that switching is a decision, not a flip-flop.
                    other = self._pick(mask, desired_deg)
                    if other is None:
                        self._choose_escape_side(cost)
                        self._enter(AvoidState.BLOCKED, now)
                    else:
                        self._commit(other, now)
                        self.state_since = now
                else:
                    self._choose_escape_side(cost)
                    self._enter(AvoidState.BLOCKED, now)

        elif self.state == AvoidState.CLEARING:
            if not path_clear:
                self.all_clear_since = None
                self._enter(AvoidState.AVOIDING, now)
            elif not view_clear:
                # Corridor is open but the obstacle is still off to the side.
                # Keep turning away from it -- this is what actually opens up
                # lateral distance, and stopping here is what made the robot
                # graze obstacles it had nominally already avoided.
                self.all_clear_since = None
            else:
                if self.all_clear_since is None:
                    self.all_clear_since = now
                elif now - self.all_clear_since >= self.clear_hold_s:
                    self.commit_sign = 0
                    self.commit_deg = 0.0
                    self.all_clear_since = None
                    self._enter(AvoidState.CLEAR, now)

        elif self.state == AvoidState.BLOCKED:
            if path_clear:
                self.commit_sign = 0
                self._enter(AvoidState.CLEAR, now)
            else:
                idx = self._pick(mask, desired_deg)
                if idx is not None:
                    self._commit(idx, now)
                elif elapsed >= self.blocked_hold_s:
                    self._choose_escape_side(cost)
                    self._enter(AvoidState.ESCAPE, now)

        elif self.state == AvoidState.ESCAPE:
            if path_clear:
                self.commit_sign = 0
                self._enter(AvoidState.CLEAR, now)
            else:
                idx = self._pick(mask, desired_deg)
                if idx is not None:
                    self._commit(idx, now)
                elif elapsed >= self.escape_max_s:
                    # The arc found nothing. Halt and re-observe rather than
                    # circling indefinitely.
                    self._enter(AvoidState.BLOCKED, now)

        # --- output for whatever state we ended up in -----------------------
        if self.state == AvoidState.CLEAR:
            return Decision(desired_deg, 1.0, AvoidState.CLEAR, "corridor clear")

        if self.state == AvoidState.AVOIDING:
            return Decision(self.commit_deg, AVOID_STRIDE_SCALE, AvoidState.AVOIDING,
                            f"gap at {self.commit_deg:+.0f} deg")

        if self.state == AvoidState.CLEARING:
            if not view_clear:
                # Still in frame off to one side: keep turning away to open up
                # lateral distance.
                return Decision(self.commit_deg, AVOID_STRIDE_SCALE,
                                AvoidState.CLEARING, "widening the gap")
            # Out of frame, but the body is ~95 cm long behind a nose-mounted
            # camera, so hold straight rather than steering back toward the
            # waypoint -- this is the window where turning back would drag a
            # rear leg into what the front just cleared.
            return Decision(0.0, AVOID_STRIDE_SCALE, AvoidState.CLEARING,
                            "driving body past obstacle")

        if self.state == AvoidState.ESCAPE:
            return Decision(self.escape_deg, ESCAPE_STRIDE_SCALE, AvoidState.ESCAPE,
                            "no gap -- arcing to find one")

        return Decision(0.0, HALT_STRIDE_SCALE, AvoidState.BLOCKED,
                        "fully blocked -- holding")


# =============================================================================
# Threaded front end
# =============================================================================

class ObstacleAvoider:
    """Runs depth inference off the control loop and answers steering queries.

    Frames arrive from whatever already owns the camera (only one process can
    hold the V4L2 device, and pi5_main's camera_loop has it). The worker takes
    the newest frame available and drops the rest -- at 1-3 fps against a 30 fps
    feed almost every frame is skipped, which is correct: stale frames are worth
    less than the newest one.
    """

    def __init__(self, model_path=None, input_size=INPUT_SIZE, logger=None):
        self.log = logger or (lambda msg: print(f"[VISION] {msg}"))
        self._lock = threading.Lock()
        self._pending = None            # newest unprocessed frame
        self._cost = None
        self._cost_time = 0.0
        self._infer_ms = 0.0
        self._last_decision = None
        self._running = False
        self._thread = None

        self.planner = AvoidancePlanner()
        self.depth = None

        path = self._resolve_model(model_path)
        if path is None:
            self.log("no depth model found -- avoidance unavailable. Fetch it "
                     f"with: python vision_obstacle.py --download\n  ({MODEL_URL})")
            return
        try:
            self.depth = DepthEstimator(path, input_size=input_size)
            self.log(f"depth model loaded: {os.path.basename(path)}")
        except Exception as e:
            self.log(f"failed to load depth model ({e}) -- avoidance unavailable")
            self.depth = None

    @staticmethod
    def _resolve_model(model_path):
        if model_path:
            return model_path if os.path.exists(model_path) else None
        for candidate in DEFAULT_MODEL_PATHS:
            if os.path.exists(candidate):
                return candidate
        return None

    @property
    def available(self):
        return self.depth is not None

    # -- lifecycle ----------------------------------------------------------

    def start(self):
        if not self.available or self._running:
            return False
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self.log("inference worker started")
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def submit_frame(self, frame):
        """Called from the camera loop. Cheap: stores a reference and returns.

        cv2's read() hands back a freshly allocated array each call, so there is
        nothing to copy -- the worker cannot be reading a buffer the camera is
        simultaneously overwriting.
        """
        if not self.available:
            return
        with self._lock:
            self._pending = frame

    def _worker(self):
        while self._running:
            with self._lock:
                frame, self._pending = self._pending, None
            if frame is None:
                time.sleep(0.02)
                continue
            t0 = time.time()
            depth = self.depth.infer(frame)
            if depth is None:
                continue
            cost = costmap_from_depth(depth)
            with self._lock:
                self._cost = cost
                self._cost_time = time.monotonic()
                self._infer_ms = (time.time() - t0) * 1000.0

    # -- queries ------------------------------------------------------------

    def snapshot(self):
        """(costmap or None, age_seconds, inference_ms). Thread safe."""
        with self._lock:
            if self._cost is None:
                return None, float("inf"), self._infer_ms
            return self._cost.copy(), time.monotonic() - self._cost_time, self._infer_ms

    def plan(self, desired_deg):
        """Steering decision for this control tick. Safe to call at 100 Hz --
        it only reads the latest costmap and advances the state machine's
        timers, it never runs the model.

        Call this from ONE thread only (the control loop). The planner's timers
        are not lock-protected, and a second caller would advance them behind
        the first one's back. The camera thread reads the result via annotate()
        instead of calling plan() itself.
        """
        if not self.available:
            return Decision(desired_deg, 1.0, AvoidState.OFF, "model unavailable",
                            fresh=False)
        cost, age, _ = self.snapshot()
        decision = self.planner.update(cost, desired_deg, fresh=age <= STALE_AFTER_S)
        with self._lock:
            self._last_decision = decision
        return decision

    # -- dashboard overlay ---------------------------------------------------

    def annotate(self, frame, decision=None):
        """Frame with the ROI bins and current decision drawn on, for the video
        feed. Returns the ORIGINAL object untouched when there is nothing to
        draw, so the normal path costs nothing.

        Safe to call from the camera thread: it only reads state, and defaults
        to whatever the control loop's last plan() produced.
        """
        if cv2 is None or not self.available:
            return frame
        cost, age, ms = self.snapshot()
        if cost is None or age > STALE_AFTER_S:
            return frame
        if decision is None:
            with self._lock:
                decision = self._last_decision
        try:
            out = frame.copy()
            h, w = out.shape[:2]
            y0, y1 = int(h * ROI_TOP), int(h * ROI_BOTTOM)
            for i, c in enumerate(cost):
                x0, x1 = int(i * w / len(cost)), int((i + 1) * w / len(cost))
                colour = (0, 0, 255) if c >= BLOCK_FRAC else (0, 200, 0)
                cv2.rectangle(out, (x0, y0), (x1, y1), colour, 2)
            if decision is not None:
                label = (f"{decision.state} {decision.steer_deg:+.0f}deg "
                         f"x{decision.stride_scale:.2f}  {ms:.0f}ms")
                cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)
            return out
        except Exception:
            return frame


# =============================================================================
# Standalone use -- no ROS, no robot
# =============================================================================

def _download(url, dest):
    import urllib.request
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    print(f"GET {url}\n -> {dest}")
    req = urllib.request.Request(url, headers={"User-Agent": "apex-vision/1.0"})
    with urllib.request.urlopen(req) as r, open(dest, "wb") as f:
        total, got = int(r.headers.get("Content-Length", 0)), 0
        while True:
            chunk = r.read(1 << 20)
            if not chunk:
                break
            f.write(chunk)
            got += len(chunk)
            if total:
                print(f"\r  {got / 1e6:6.1f} / {total / 1e6:.1f} MB", end="")
    print(f"\r  saved ({os.path.getsize(dest) / 1e6:.1f} MB)          ")


def _synthetic_scenes(h=480, w=640, horizon_frac=0.42):
    """Depth maps with the structure a real forward camera produces.

    Inverse depth is linear in row below the horizon (see costmap_from_depth),
    and an upright object sits at the ground's depth where its feet touch --
    which is nearer than the ground reads at every row above. Enough to catch a
    costmap that cannot tell ground from obstacle.
    """
    horizon = int(h * horizon_frac)

    def ground():
        d = np.zeros((h, w), np.float32)
        rows = np.arange(h)
        below = rows > horizon
        d[below] = ((rows[below] - horizon) / (h - horizon))[:, None]
        return d

    def upright(d, cx, half_w, feet_row, top_row):
        d = d.copy()
        d[top_row:feet_row, max(0, cx - half_w):min(w, cx + half_w)] = \
            (feet_row - horizon) / (h - horizon)
        return d

    return [
        ("open ground", ground(), lambda c: not (c >= BLOCK_FRAC).any()),
        ("figure dead centre", upright(ground(), w // 2, 38, 400, 150),
         lambda c: c[N_BINS // 2] >= BLOCK_FRAC and not (c[:2] >= BLOCK_FRAC).any()
         and not (c[-2:] >= BLOCK_FRAC).any()),
        ("figure on the right", upright(ground(), int(w * 0.83), 38, 400, 150),
         lambda c: (c[-3:] >= BLOCK_FRAC).any() and not (c[:3] >= BLOCK_FRAC).any()),
        ("wall filling the view", np.full((h, w), 0.62, np.float32),
         lambda c: (c >= BLOCK_FRAC).all()),
    ]


def _selftest():
    """Planner logic against hand-built costmaps. No model, no camera."""
    print("costmap, against synthetic ground-plane scenes:")
    for name, depth, want in _synthetic_scenes():
        cost = costmap_from_depth(depth)
        bars = "".join("#" if c >= BLOCK_FRAC else "." for c in cost)
        print(f"  {'ok ' if want(cost) else 'BAD'} {name:22s} [{bars}]  "
              + " ".join(f"{c:.2f}" for c in cost))
    print("\nplanner:")
    p = AvoidancePlanner()
    clear = np.zeros(N_BINS, np.float32)
    wall_right = np.array([0, 0, 0, 0, .8, .9, .9, .9, .9], np.float32)
    wall_left = wall_right[::-1].copy()
    wall_all = np.full(N_BINS, 0.9, np.float32)

    t = 0.0
    checks = [
        (clear, 0.0, "clear ahead", AvoidState.CLEAR),
        (wall_right, 0.0, "wall on the right", AvoidState.AVOIDING),
        (wall_right, 40.0, "wall right, goal right", AvoidState.AVOIDING),
        (clear, 0.0, "obstacle passed", AvoidState.CLEARING),
    ]
    for cost, desired, label, expect in checks:
        t += 0.1
        d = p.update(cost, desired, now=t)
        flag = "ok " if d.state == expect else "BAD"
        print(f"  {flag} {label:26s} -> {d.state:9s} steer {d.steer_deg:+6.1f} "
              f"stride {d.stride_scale:.2f}  ({d.reason})")

    # The release timer starts on the first sample where the whole view is
    # clear -- which is the pass AFTER the one that entered CLEARING -- so
    # reaching CLEAR takes one sample to arm the timer and one to expire it.
    t += 0.1
    d = p.update(clear, 0.0, now=t)
    print(f"  {'ok ' if d.state == AvoidState.CLEARING else 'BAD'} "
          f"{'hold timer arms, still held':26s} -> {d.state:9s}")
    t += CLEAR_HOLD_S + 0.1
    d = p.update(clear, 0.0, now=t)
    print(f"  {'ok ' if d.state == AvoidState.CLEAR else 'BAD'} "
          f"{'after clear-hold expires':26s} -> {d.state:9s} steer {d.steer_deg:+6.1f}")

    # Turning back must NOT happen while the obstacle is still in frame off to
    # the side -- that is what makes the robot graze what it just avoided.
    p.reset()
    t += 1.0
    p.update(wall_right, 0.0, now=t)                 # commit left
    t += 0.1
    edge_only = np.array([0, 0, 0, 0, 0, 0, 0, .8, .9], np.float32)
    d = p.update(edge_only, 0.0, now=t)              # corridor clear, still in view
    still_turning = d.state == AvoidState.CLEARING and d.steer_deg < -1
    print(f"  {'ok ' if still_turning else 'BAD'} "
          f"{'keeps turning while in view':26s} -> {d.state:9s} steer {d.steer_deg:+6.1f}")

    p.reset()
    t += 1.0
    d = p.update(wall_all, 0.0, now=t)
    print(f"  {'ok ' if d.state == AvoidState.BLOCKED else 'BAD'} "
          f"{'wall everywhere':26s} -> {d.state:9s} stride {d.stride_scale:.2f}")
    t += BLOCKED_HOLD_S + 0.1
    d = p.update(wall_all, 0.0, now=t)
    print(f"  {'ok ' if d.state == AvoidState.ESCAPE else 'BAD'} "
          f"{'still blocked -> escape':26s} -> {d.state:9s} steer {d.steer_deg:+6.1f}")

    # Anti-oscillation: the classic trap is an obstacle sitting exactly between
    # the robot and the waypoint, so nav keeps pointing back into it.
    p.reset()
    t += 1.0
    p.update(wall_left, 0.0, now=t)
    sides = set()
    for k in range(20):
        t += 0.1
        d = p.update(wall_left, -30.0, now=t)   # nav insists on going left
        sides.add(1 if d.steer_deg >= 0 else -1)
    print(f"  {'ok ' if len(sides) == 1 else 'BAD'} "
          f"{'no flip-flop under nav':26s} -> committed to {sides}")


def _live(device):
    cam = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cam.isOpened():
        print(f"could not open {device}")
        return

    avoider = ObstacleAvoider()
    if not avoider.start():
        print("model unavailable; run with --download first")
        cam.release()
        return

    print("streaming decisions; ctrl-c to stop")
    try:
        while True:
            ok, frame = cam.read()
            if not ok:
                continue
            avoider.submit_frame(frame)
            d = avoider.plan(0.0)
            cost, age, ms = avoider.snapshot()
            bars = "".join("#" if c >= BLOCK_FRAC else "." for c in cost) if cost is not None else "?"
            print(f"\r[{bars}] {d.state:9s} steer {d.steer_deg:+6.1f} "
                  f"stride {d.stride_scale:.2f}  {ms:5.0f}ms age {age:4.1f}s   ", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        avoider.stop()
        cam.release()


if __name__ == "__main__":
    import sys

    args = sys.argv[1:]
    if "--download" in args:
        _download(MODEL_URL, DEFAULT_MODEL_PATHS[0])
    elif "--live" in args:
        idx = args.index("--live")
        device = args[idx + 1] if len(args) > idx + 1 else 0
        _live(device)
    else:
        print("planner self-test (no model, no camera):")
        _selftest()
        print("\nother modes:  --download   fetch the depth model"
              "\n              --live [dev] run against the camera")
