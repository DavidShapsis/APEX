"""
dashboard_preview.py  --  TEMPORARY, throwaway UI preview. Not part of the robot.
=================================================================================

A standalone copy of the stream_server.py dashboard with every integration
removed: no ROS 2, no OpenCV, no serial, no camera, no vision model. The only
dependency is Flask.

    pip install flask
    python3 dashboard_preview.py
    -> http://localhost:5000

The page markup, CSS and JavaScript are copied verbatim from stream_server.py so
what you see here is what the robot serves. What is faked:

  * ROS publishers          -> mutate a MockRobot in memory
  * /video_feed             -> an animated SVG placeholder, so there is
                               something in the frame and the ROI-bin overlay
                               is visible without a camera
  * pi5_main's status topic -> MockRobot.status(), same 19-field layout

The mock also runs the real gating rules, so the Home -> Stand -> Go -> Stop
flow behaves the way the robot does, including STAND-while-walking settling
rather than being refused. Delete this file when you are done looking at it.
"""

import threading
import time

from flask import Flask, Response, jsonify, request

import dashboard_page

LEG_NAMES = ['FL', 'FR', 'RR', 'RL']

# Must match AvoidState.CODES in vision_obstacle.py.
AVOID_CLEAR, AVOID_AVOIDING, AVOID_CLEARING, AVOID_BLOCKED, AVOID_ESCAPE = 1, 2, 3, 4, 5


class MockRobot:
    """Stands in for pi5_main. Holds the same state and enforces the same
    gating, so the button flow is exercised rather than just rendered."""

    def __init__(self):
        self.lock = threading.Lock()
        self.homed = [0, 0, 0, 0]
        self.standing = 0
        self.walking = 0
        self.deactivated = [0, 0, 0, 0]

        # Pretend the depth model loaded, otherwise the avoidance button stays
        # disabled and there is nothing to look at. Flip to 0 to preview the
        # "NO MODEL" state instead.
        self.avoid_available = 1
        self.avoid_enabled = 0
        self.avoid_state = AVOID_CLEAR
        self.avoid_steer = 0
        self.avoid_stride = 100

        self.direction = 0
        self.nav_mode = False

        # Route: [[lat, lon], ...] plus which one we are 'driving to'. The
        # _route_sim thread below advances the index while walking, in NAV mode,
        # not paused.
        self.waypoints = []
        self.wp_index = 0
        self.nav_paused = False

        threading.Thread(target=self._avoid_sim, daemon=True).start()
        threading.Thread(target=self._route_sim, daemon=True).start()

    # -- the real gating rules, reproduced ---------------------------------

    def home_leg(self, leg_id):
        with self.lock:
            self.homed[leg_id] = 1

    def stand(self):
        with self.lock:
            if not all(self.homed):
                return False, "Home every leg before Stand."
            # Matches request_stand(): while walking this is a settle, not a
            # ramp from HOME_POSE.
            self.standing, self.walking = 1, 0
            return True, None

    def go(self):
        with self.lock:
            if not (all(self.homed) and self.standing):
                return False, "Home every leg and Stand before Go."
            self.walking = 1
            return True, None

    def stop(self):
        with self.lock:
            self.standing, self.walking = 0, 0

    def set_deactivated(self, leg_id, off):
        with self.lock:
            self.deactivated[leg_id] = 1 if off else 0

    def toggle_avoid(self):
        with self.lock:
            if not self.avoid_available:
                return False, "Vision model not loaded on the Pi."
            self.avoid_enabled = 0 if self.avoid_enabled else 1
            if not self.avoid_enabled:
                self.avoid_state, self.avoid_steer, self.avoid_stride = \
                    AVOID_CLEAR, 0, 100
            return True, None

    def status(self):
        with self.lock:
            return {
                'homed': list(self.homed),
                'standing': self.standing,
                'walking': self.walking,
                'deactivated': list(self.deactivated),
                'avoid_available': self.avoid_available,
                'avoid_enabled': self.avoid_enabled,
                'avoid_state': self.avoid_state,
                'avoid_steer': self.avoid_steer,
                'avoid_stride': self.avoid_stride,
                'wp_index': self.wp_index,
                'wp_total': len(self.waypoints),
                'nav_mode': 1 if self.nav_mode else 0,
                'nav_paused': 1 if self.nav_paused else 0,
            }

    def set_waypoints(self, pairs):
        with self.lock:
            self.waypoints = [[float(a), float(b)] for a, b in pairs]
            self.wp_index = 0

    def nav_control(self, action):
        with self.lock:
            if action == 'start':
                self.nav_mode, self.nav_paused, self.wp_index = True, False, 0
            elif action == 'pause':
                self.nav_paused = True
            elif action == 'stop':
                self.nav_mode, self.nav_paused, self.wp_index = False, False, 0

    # -- fake perception ----------------------------------------------------

    def _avoid_sim(self):
        """Walks the avoidance state machine through a plausible detour so all
        five states, colours and stride values actually appear in the readout.
        Only runs while walking with avoidance on."""
        script = [
            (AVOID_CLEAR,    0,   100, 6),
            (AVOID_AVOIDING, -35,  60, 5),
            (AVOID_AVOIDING, -20,  60, 3),
            (AVOID_CLEARING, -35,  60, 4),
            (AVOID_CLEARING,   0,  60, 4),
            (AVOID_CLEAR,      0, 100, 5),
            (AVOID_BLOCKED,    0,   0, 3),
            (AVOID_ESCAPE,   +45,  35, 4),
            (AVOID_CLEAR,      0, 100, 6),
        ]
        i = 0
        while True:
            with self.lock:
                live = self.avoid_enabled and self.walking
            if not live:
                time.sleep(0.5)
                continue
            state, steer, stride, hold = script[i % len(script)]
            with self.lock:
                self.avoid_state, self.avoid_steer, self.avoid_stride = \
                    state, steer, stride
            i += 1
            time.sleep(hold)

    def _route_sim(self):
        """Advance through the loaded route while walking in NAV mode, so the
        'driving to waypoint N of M' / 'route complete' readout is exercised."""
        while True:
            time.sleep(4.0)
            with self.lock:
                if (self.nav_mode and self.walking and not self.nav_paused
                        and self.wp_index < len(self.waypoints)):
                    self.wp_index += 1


robot = MockRobot()
app = Flask(__name__)


# ==============================================================================
# Placeholder video: an SVG that mimics a ground-plane view with the ROI bins
# drawn on, so the overlay is previewable without a camera or a depth model.
# ==============================================================================

def _feed_svg():
    s = robot.status()
    on = s['avoid_enabled'] == 1
    state = s['avoid_state']
    # Which bins read blocked, per state -- purely cosmetic, to show the overlay.
    blocked = {
        AVOID_CLEAR:    set(),
        AVOID_AVOIDING: {4, 5, 6},
        AVOID_CLEARING: {6, 7},
        AVOID_BLOCKED:  set(range(9)),
        AVOID_ESCAPE:   {0, 1, 2, 3, 4, 5},
    }.get(state, set())

    W, H, N = 640, 400, 9
    y0, y1 = int(H * 0.45), int(H * 0.95)
    bins = ""
    if on:
        for i in range(N):
            x = i * W / N
            col = "#ff2020" if i in blocked else "#00c800"
            bins += (f'<rect x="{x:.1f}" y="{y0}" width="{W/N:.1f}" '
                     f'height="{y1-y0}" fill="none" stroke="{col}" '
                     f'stroke-width="3"/>')

    label = ""
    if on:
        names = {AVOID_CLEAR: "CLEAR", AVOID_AVOIDING: "AVOIDING",
                 AVOID_CLEARING: "CLEARING", AVOID_BLOCKED: "BLOCKED",
                 AVOID_ESCAPE: "ESCAPE"}
        label = (f'<text x="12" y="30" fill="#ffff00" font-family="monospace" '
                 f'font-size="20">{names.get(state, "")} '
                 f'{s["avoid_steer"]:+d}deg x{s["avoid_stride"]/100:.2f}</text>')

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}"
     viewBox="0 0 {W} {H}">
  <defs>
    <linearGradient id="sky" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#4a6fa5"/><stop offset="100%" stop-color="#9fb8d4"/>
    </linearGradient>
    <linearGradient id="ground" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#5c6b3f"/><stop offset="100%" stop-color="#8a7f5c"/>
    </linearGradient>
  </defs>
  <rect width="{W}" height="{int(H*0.42)}" fill="url(#sky)"/>
  <rect y="{int(H*0.42)}" width="{W}" height="{H}" fill="url(#ground)"/>
  <polygon points="{W/2-18},{H*0.42} {W/2+18},{H*0.42} {W*0.78},{H} {W*0.22},{H}"
           fill="#b9ad8c" opacity="0.85"/>
  <ellipse cx="{W*0.5}" cy="{H*0.72}" rx="26" ry="54" fill="#3a3a3a" opacity="0.9">
    <animate attributeName="cx" values="{W*0.5};{W*0.62};{W*0.5}"
             dur="9s" repeatCount="indefinite"/>
  </ellipse>
  {bins}
  {label}
  <text x="12" y="{H-14}" fill="#ffffff" font-family="monospace" font-size="15"
        opacity="0.75">PREVIEW -- no camera attached</text>
</svg>"""


@app.route('/video_feed')
def video_feed():
    # A plain SVG, not an MJPEG stream: no encoder, no camera, and the browser
    # still animates it. The real server sends multipart/x-mixed-replace here.
    return Response(_feed_svg(), mimetype='image/svg+xml',
                    headers={'Cache-Control': 'no-store'})


# ==============================================================================
# Routes -- same paths and same JSON shapes as stream_server.py
# ==============================================================================

@app.route('/set_direction', methods=['POST'])
def set_direction():
    robot.direction = int(request.form.get('angle', 0))
    return "OK"


@app.route('/toggle_nav', methods=['POST'])
def toggle_nav():
    robot.nav_mode = not robot.nav_mode
    return jsonify({"nav_mode": robot.nav_mode})


@app.route('/toggle_avoid', methods=['POST'])
def toggle_avoid():
    ok, err = robot.toggle_avoid()
    if not ok:
        return jsonify({"ok": False, "avoid_mode": False, "error": err}), 409
    return jsonify({"ok": True, "avoid_mode": bool(robot.avoid_enabled)})


def _leg_id():
    try:
        leg_id = int(request.form.get('leg', -1))
    except ValueError:
        leg_id = -1
    return leg_id if leg_id in (0, 1, 2, 3) else None


@app.route('/home_leg', methods=['POST'])
def home_leg():
    leg_id = _leg_id()
    if leg_id is None:
        return jsonify({"ok": False, "error": "bad leg id"}), 400
    robot.home_leg(leg_id)
    return jsonify({"ok": True})


@app.route('/stand', methods=['POST'])
def stand():
    ok, err = robot.stand()
    return (jsonify({"ok": True}) if ok
            else (jsonify({"ok": False, "error": err}), 409))


@app.route('/go', methods=['POST'])
def go():
    ok, err = robot.go()
    return (jsonify({"ok": True}) if ok
            else (jsonify({"ok": False, "error": err}), 409))


@app.route('/stop', methods=['POST'])
def stop():
    robot.stop()
    return jsonify({"ok": True})


@app.route('/deactivate_leg', methods=['POST'])
def deactivate_leg():
    leg_id = _leg_id()
    if leg_id is None:
        return jsonify({"ok": False, "error": "bad leg id"}), 400
    robot.set_deactivated(leg_id, True)
    return jsonify({"ok": True})


@app.route('/reactivate_leg', methods=['POST'])
def reactivate_leg():
    leg_id = _leg_id()
    if leg_id is None:
        return jsonify({"ok": False, "error": "bad leg id"}), 400
    robot.set_deactivated(leg_id, False)
    return jsonify({"ok": True})


@app.route('/status')
def status():
    return jsonify(robot.status())


@app.route('/waypoints')
def get_waypoints():
    return jsonify({"waypoints": robot.waypoints})


@app.route('/nav_control', methods=['POST'])
def nav_control():
    action = request.form.get('action', '')
    if action not in ('start', 'pause', 'stop'):
        return jsonify({"ok": False, "error": "action must be start, pause or stop"}), 400
    robot.nav_control(action)
    return jsonify({"ok": True, "action": action})


@app.route('/set_waypoints', methods=['POST'])
def set_waypoints():
    raw = (request.get_json(silent=True) or {}).get("waypoints", [])
    clean = []
    for pair in raw:
        try:
            lat, lon = float(pair[0]), float(pair[1])
        except (TypeError, ValueError, IndexError):
            return jsonify({"ok": False, "error": "Every point needs a numeric latitude and longitude."}), 400
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return jsonify({"ok": False, "error": f"({lat}, {lon}) is outside valid latitude/longitude range."}), 400
        clean.append([lat, lon])
    robot.set_waypoints(clean)
    return jsonify({"ok": True, "count": len(clean)})


# ==============================================================================
# The page -- copied verbatim from stream_server.py's index()
# ==============================================================================

_PREVIEW_REPAINT = """
<script>
    // PREVIEW ONLY: the real /video_feed is an MJPEG stream that pushes
    // frames itself. This mock serves a still SVG, so poke it once a second
    // to repaint and pick up the current avoidance overlay.
    setInterval(() => {
        const img = document.querySelector("img[src^='/video_feed']");
        if (img) img.src = '/video_feed?t=' + Date.now();
    }, 1000);
</script>
"""


@app.route('/')
def index():
    nav_btn_color = "#ff0000" if not robot.nav_mode else "#00ff00"
    nav_text = "NAV MODE: OFF" if not robot.nav_mode else "NAV MODE: ON"
    return dashboard_page.render(nav_text, nav_btn_color,
                                 extra_body=_PREVIEW_REPAINT)


if __name__ == '__main__':
    print("=" * 68)
    print("  APEX dashboard PREVIEW  --  mock robot, no hardware")
    print("=" * 68)
    print("  http://localhost:5000")
    print()
    print("  Try:  Home FL/FR/RR/RL  ->  STAND  ->  GO")
    print("        then AVOIDANCE: ON to watch it cycle CLEAR -> AVOIDING ->")
    print("        CLEARING -> BLOCKED -> ESCAPE, with the bin overlay on the feed.")
    print("        STAND while WALKING settles instead of refusing.")
    print("        Set MockRobot.avoid_available = 0 to preview the NO MODEL state.")
    print("=" * 68)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True,
            use_reloader=False)
