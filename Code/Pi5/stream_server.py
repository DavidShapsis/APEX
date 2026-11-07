import json
import time
import cv2
import threading
from flask import Flask, Response, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray, Bool

import dashboard_page

LEG_NAMES = ['FL', 'FR', 'RR', 'RL']

class RobodogStreamer(Node):
    def __init__(self, host='0.0.0.0', port=5000):
        super().__init__('stream_server_node')

        # ROS 2 Publishers
        self.dir_pub = self.create_publisher(Int32, '/apex/navigation/cmd_dir', 10)
        self.nav_mode_pub = self.create_publisher(Bool, '/apex/navigation/nav_mode', 10)
        self.avoid_pub = self.create_publisher(Bool, '/apex/vision/cmd_avoid', 10)
        self.home_leg_pub = self.create_publisher(Int32, '/apex/homing/cmd_home_leg', 10)
        self.stand_pub = self.create_publisher(Bool, '/apex/homing/cmd_stand', 10)
        self.go_pub = self.create_publisher(Bool, '/apex/homing/cmd_go', 10)
        self.stop_pub = self.create_publisher(Bool, '/apex/homing/cmd_stop', 10)
        self.deactivate_leg_pub = self.create_publisher(Int32, '/apex/homing/cmd_deactivate_leg', 10)
        self.reactivate_leg_pub = self.create_publisher(Int32, '/apex/homing/cmd_reactivate_leg', 10)
        self.waypoints_pub = self.create_publisher(
            Float32MultiArray, '/apex/navigation/waypoints', 10)
        self.nav_cmd_pub = self.create_publisher(
            Int32, '/apex/navigation/nav_cmd', 10)
        self.homing_status_sub = self.create_subscription(
            Int32MultiArray, '/apex/homing/status', self.homing_status_callback, 10)

        self.app = Flask(__name__)
        self.host = host
        self.port = port
        self.output_frame = None
        self.frame_seq = 0          # bumps on every new camera frame
        self.frame_time = 0.0       # time.monotonic() of the newest frame
        self.lock = threading.Lock()
        self.started = time.monotonic()     # for /api/state uptime

        self.current_direction = 0
        self.nav_mode = False  # Track state of autonomous navigation
        self.avoid_mode = False  # Track state of vision obstacle avoidance

        # The route lives on the controller (Navigator). This is the browser's
        # working copy: what the operator has typed but not necessarily sent.
        # Seeded from the controller's echo in the status message so a page
        # reload shows the route that is actually loaded, not an empty list.
        self.waypoints = []

        # Updated by homing_status_callback from pi5_main.py's periodic publish.
        # Everything starts false/0 -- matches the real state at boot, since
        # nothing is homed until the operator does it from this UI.
        self.homing_status = {
            'homed': [0, 0, 0, 0], 'standing': 0, 'walking': 0,
            'deactivated': [0, 0, 0, 0],
            'avoid_available': 0, 'avoid_enabled': 0, 'avoid_state': 0,
            'avoid_steer': 0, 'avoid_stride': 100,
            'wp_index': 0, 'wp_total': 0, 'nav_mode': 0, 'nav_paused': 0,
            'health': 0x3F,   # 6 bits, all subsystems assumed up until told otherwise
            'currents': [-1] * 24,
            'gps': {'fix': 0, 'sats': 0, 'lat': None, 'lon': None, 'heading': None},
            'power': {'valid': 0, 'amps': None, 'volts': None},
        }

        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/set_direction', 'set_direction', self.set_direction, methods=['POST'])
        self.app.add_url_rule('/toggle_nav', 'toggle_nav', self.toggle_nav, methods=['POST'])
        self.app.add_url_rule('/toggle_avoid', 'toggle_avoid', self.toggle_avoid, methods=['POST'])
        self.app.add_url_rule('/home_leg', 'home_leg', self.home_leg, methods=['POST'])
        self.app.add_url_rule('/stand', 'stand', self.stand, methods=['POST'])
        self.app.add_url_rule('/go', 'go', self.go, methods=['POST'])
        self.app.add_url_rule('/stop', 'stop', self.stop, methods=['POST'])
        self.app.add_url_rule('/deactivate_leg', 'deactivate_leg', self.deactivate_leg, methods=['POST'])
        self.app.add_url_rule('/reactivate_leg', 'reactivate_leg', self.reactivate_leg, methods=['POST'])
        self.app.add_url_rule('/status', 'status', self.status, methods=['GET'])
        self.app.add_url_rule('/api/state', 'api_state', self.api_state, methods=['GET'])
        self.app.add_url_rule('/waypoints', 'get_waypoints', self.get_waypoints, methods=['GET'])
        self.app.add_url_rule('/set_waypoints', 'set_waypoints', self.set_waypoints, methods=['POST'])
        self.app.add_url_rule('/nav_control', 'nav_control', self.nav_control, methods=['POST'])

    def homing_status_callback(self, msg):
        data = list(msg.data)
        if len(data) >= 10:
            status = {
                'homed': data[0:4],
                'standing': data[4],
                'walking': data[5],
                'deactivated': data[6:10],
                # Defaults, so a controller that predates the avoidance fields
                # still produces a complete status dict for the UI.
                'avoid_available': 0, 'avoid_enabled': 0, 'avoid_state': 0,
                'avoid_steer': 0, 'avoid_stride': 100,
                'wp_index': 0, 'wp_total': 0, 'nav_mode': 0, 'nav_paused': 0,
                'health': 0x3F,
                'currents': [-1] * 24,
                'gps': {'fix': 0, 'sats': 0, 'lat': None, 'lon': None, 'heading': None},
                'power': {'valid': 0, 'amps': None, 'volts': None},
            }
            if len(data) >= 15:
                status.update({
                    'avoid_available': data[10],
                    'avoid_enabled': data[11],
                    'avoid_state': data[12],
                    'avoid_steer': data[13],
                    'avoid_stride': data[14],
                })
                # The controller is the authority on whether avoidance is
                # actually on, so a toggle lost in transit self-corrects here
                # instead of leaving the button lying about the robot's state.
                self.avoid_mode = bool(data[11])
            if len(data) >= 17:
                status['wp_index'] = data[15]
                status['wp_total'] = data[16]
            if len(data) >= 19:
                status['nav_mode'] = data[17]
                status['nav_paused'] = data[18]
                # Controller is the authority; keep the toggle honest even if a
                # /toggle_nav or Start/Stop was lost in transit.
                self.nav_mode = bool(data[17])
            if len(data) >= 20:
                status['health'] = data[19]
            if len(data) >= 44:
                # mA, [leg FL,FR,RR,RL][joint roll,pitch,knee][R,L]; -1 = no data.
                status['currents'] = data[20:44]
            if len(data) >= 49:
                fix, sats, lat_e7, lon_e7, head_x10 = data[44:49]
                status['gps'] = {
                    'fix': fix, 'sats': sats,
                    'lat': lat_e7 / 1e7 if fix else None,
                    'lon': lon_e7 / 1e7 if fix else None,
                    'heading': head_x10 / 10 if head_x10 >= 0 else None,
                }
            if len(data) >= 52:
                p_valid, p_ma, p_mv = data[49:52]
                status['power'] = {
                    'valid': p_valid,
                    'amps': p_ma / 1000.0 if p_valid else None,
                    'volts': p_mv / 1000.0 if p_valid else None,
                }
            self.homing_status = status

    def index(self):
        nav_btn_color = "#ff0000" if not self.nav_mode else "#00ff00"
        nav_text = "NAV MODE: OFF" if not self.nav_mode else "NAV MODE: ON"
        return dashboard_page.render(nav_text, nav_btn_color)

    def set_direction(self):
        self.current_direction = int(request.form.get('angle', 0))
        msg = Int32()
        msg.data = self.current_direction
        self.dir_pub.publish(msg)
        return "OK"

    def toggle_nav(self):
        """Toggles navigation mode state and publishes it to ROS 2."""
        going_on = not self.nav_mode
        if going_on and not self.homing_status.get('wp_total', 0):
            # The controller rejects AUTONOMOUS with an empty route; refuse here
            # too so the button does not flip on and bounce straight back.
            return jsonify({"ok": False, "nav_mode": self.nav_mode,
                            "error": "Add at least one waypoint before NAV MODE."}), 409
        self.nav_mode = going_on
        msg = Bool()
        msg.data = self.nav_mode
        self.nav_mode_pub.publish(msg)
        return jsonify({"ok": True, "nav_mode": self.nav_mode})

    def nav_control(self):
        """Route transport buttons. Body: form field action=start|pause|stop.
        Maps to nav_cmd 1/2/3 on the controller."""
        action = request.form.get('action', '')
        code = {'start': 1, 'pause': 2, 'stop': 3}.get(action)
        if code is None:
            return jsonify({"ok": False, "error": "action must be start, pause or stop"}), 400
        if action == 'start' and not self.homing_status.get('wp_total', 0):
            return jsonify({"ok": False, "error": "No route loaded. Add a waypoint and Send route first."}), 409
        msg = Int32()
        msg.data = code
        self.nav_cmd_pub.publish(msg)
        # Optimistic local mirror so the NAV MODE button reflects the press
        # before the next status frame; the status parse corrects it if wrong.
        if action == 'start':
            self.nav_mode = True
        elif action == 'stop':
            self.nav_mode = False
        return jsonify({"ok": True, "action": action})

    def toggle_avoid(self):
        """Toggles vision obstacle avoidance. Independent of nav mode -- it
        layers on top of manual steering and GPS waypoint following alike."""
        if not self.homing_status.get('avoid_available'):
            return jsonify({"ok": False, "avoid_mode": False,
                            "error": "Vision model not loaded on the Pi."}), 409
        self.avoid_mode = not self.avoid_mode
        msg = Bool()
        msg.data = self.avoid_mode
        self.avoid_pub.publish(msg)
        return jsonify({"ok": True, "avoid_mode": self.avoid_mode})

    def home_leg(self):
        try:
            leg_id = int(request.form.get('leg', -1))
        except ValueError:
            leg_id = -1
        if leg_id not in (0, 1, 2, 3):
            return jsonify({"ok": False, "error": "bad leg id"}), 400
        msg = Int32()
        msg.data = leg_id
        self.home_leg_pub.publish(msg)
        return jsonify({"ok": True})

    def stand(self):
        if not all(v == 1 for v in self.homing_status['homed']):
            return jsonify({"ok": False, "error": "Home every leg before Stand."}), 409
        msg = Bool()
        msg.data = True
        self.stand_pub.publish(msg)
        return jsonify({"ok": True})

    def go(self):
        if not (all(v == 1 for v in self.homing_status['homed']) and self.homing_status['standing'] == 1):
            return jsonify({"ok": False, "error": "Home every leg and Stand before Go."}), 409
        msg = Bool()
        msg.data = True
        self.go_pub.publish(msg)
        return jsonify({"ok": True})

    def stop(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
        return jsonify({"ok": True})

    def deactivate_leg(self):
        try:
            leg_id = int(request.form.get('leg', -1))
        except ValueError:
            leg_id = -1
        if leg_id not in (0, 1, 2, 3):
            return jsonify({"ok": False, "error": "bad leg id"}), 400
        msg = Int32()
        msg.data = leg_id
        self.deactivate_leg_pub.publish(msg)
        return jsonify({"ok": True})

    def reactivate_leg(self):
        try:
            leg_id = int(request.form.get('leg', -1))
        except ValueError:
            leg_id = -1
        if leg_id not in (0, 1, 2, 3):
            return jsonify({"ok": False, "error": "bad leg id"}), 400
        msg = Int32()
        msg.data = leg_id
        self.reactivate_leg_pub.publish(msg)
        return jsonify({"ok": True})

    def status(self):
        # frame_age lets the dashboard tell a dead camera from a dropped stream.
        with self.lock:
            t = self.frame_time
        age = round(time.monotonic() - t, 2) if t else None
        return jsonify(dict(self.homing_status, frame_age=age))

    # Names for the numeric codes in the status message. HEALTH_NAMES must match
    # HEALTH_BITS in pi5_main.py; AVOID_STATE_NAMES must match AvoidState.CODES
    # in vision_obstacle.py.
    HEALTH_NAMES = ('imu', 'gps', 'compass', 'power', 'camera', 'audio')
    JOINT_NAMES = ('roll', 'pitch', 'knee')
    AVOID_STATE_NAMES = ('off', 'clear', 'avoiding', 'clearing', 'blocked', 'escape')
    COMPASS_POINTS = ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')

    def api_state(self):
        """GET /api/state -- everything the robot reports, as one readable JSON
        document with named fields, real booleans and units in the key names
        (no bitmasks or index-into-an-array codes). Meant for clients such as
        the iOS app; /status is left unchanged for the web dashboard.

        Add ?compact=1 for a single-line response.
        """
        s = self.homing_status
        with self.lock:
            t = self.frame_time
        frame_age = round(time.monotonic() - t, 2) if t else None

        def ma(v):
            return None if v is None or v < 0 else v    # -1 = no data

        cur = list(s.get('currents') or [])
        cur += [-1] * (24 - len(cur))
        homed = list(s.get('homed') or [0, 0, 0, 0])
        deact = list(s.get('deactivated') or [0, 0, 0, 0])
        legs = []
        for i, name in enumerate(LEG_NAMES):
            legs.append({
                'name': name,
                'index': i,
                'homed': homed[i] == 1,
                'deactivated': deact[i] == 1,
                # mA; right/left side of each driver, null when not reporting.
                'currents_ma': {
                    joint: {'right': ma(cur[i * 6 + j * 2]),
                            'left': ma(cur[i * 6 + j * 2 + 1])}
                    for j, joint in enumerate(self.JOINT_NAMES)
                },
            })

        mask = s.get('health', 0x3F)
        health = {name: bool((mask >> i) & 1)
                  for i, name in enumerate(self.HEALTH_NAMES)}

        g = s.get('gps') or {}
        heading = g.get('heading')
        gps = {
            'fix': g.get('fix') == 1 and g.get('lat') is not None,
            'satellites': g.get('sats', 0),
            'lat': g.get('lat'),
            'lon': g.get('lon'),
            'heading_deg': heading,                 # true north, filtered
            'compass_point': (self.COMPASS_POINTS[int(round(heading / 45)) % 8]
                              if heading is not None else None),
        }

        p = s.get('power') or {}
        amps, volts = p.get('amps'), p.get('volts')
        power = {
            'valid': bool(p.get('valid')),
            'rail_5v_amps': amps,
            'rail_5v_volts': volts,
            'rail_5v_watts': (round(amps * volts, 3)
                              if amps is not None and volts is not None else None),
        }

        wp_total, wp_index = s.get('wp_total', 0), s.get('wp_index', 0)
        code = s.get('avoid_state', 0)
        doc = {
            'meta': {
                'schema': 1,
                'time_unix': round(time.time(), 3),
                'uptime_s': round(time.monotonic() - self.started, 1),
            },
            'robot': {
                'all_homed': all(v == 1 for v in homed),
                'standing': s.get('standing') == 1,
                'walking': s.get('walking') == 1,
                'steering_deg': self.current_direction,   # last angle sent from a client
            },
            'legs': legs,
            'health': {
                'ok': all(health.values()),
                'offline': [n for n, up in health.items() if not up],
                'subsystems': health,
                'mask': mask,
            },
            'gps': gps,
            'power': power,
            'navigation': {
                'autonomous': s.get('nav_mode') == 1,
                'paused': s.get('nav_paused') == 1,
                'waypoint_index': wp_index,
                'waypoint_total': wp_total,
                'route_complete': wp_total > 0 and wp_index >= wp_total,
                # The last route sent from the dashboard or app.
                'waypoints': [{'lat': w[0], 'lon': w[1]} for w in self.waypoints],
            },
            'avoidance': {
                'available': s.get('avoid_available') == 1,
                'enabled': s.get('avoid_enabled') == 1,
                'state': (self.AVOID_STATE_NAMES[code]
                          if 0 <= code < len(self.AVOID_STATE_NAMES) else 'unknown'),
                'state_code': code,
                'steer_deg': s.get('avoid_steer', 0),
                'stride_pct': s.get('avoid_stride', 100),
            },
            'camera': {
                'frame_age_s': frame_age,
                'live': frame_age is not None and frame_age <= 2,
            },
        }
        indent = None if request.args.get('compact') else 2
        return Response(json.dumps(doc, indent=indent) + '\n',
                        mimetype='application/json',
                        headers={'Cache-Control': 'no-store'})

    def get_waypoints(self):
        """The browser's working copy of the route. Sent on page load so a
        reload restores whatever was last entered here."""
        return jsonify({"waypoints": self.waypoints})

    def set_waypoints(self):
        """Accept an edited route from the dashboard and publish it to the
        controller. Body is JSON {"waypoints": [[lat, lon], ...]}.

        Validated here for a clean UI error; Navigator re-validates on the
        controller side because ROS messages can arrive from anywhere.
        """
        payload = request.get_json(silent=True) or {}
        raw = payload.get("waypoints", [])
        clean, flat = [], []
        for pair in raw:
            try:
                lat, lon = float(pair[0]), float(pair[1])
            except (TypeError, ValueError, IndexError):
                return jsonify({"ok": False,
                                "error": "Every point needs a numeric latitude and longitude."}), 400
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                return jsonify({"ok": False,
                                "error": f"({lat}, {lon}) is outside valid latitude/longitude range."}), 400
            clean.append([lat, lon])
            flat += [lat, lon]
        self.waypoints = clean
        msg = Float32MultiArray()
        msg.data = flat
        self.waypoints_pub.publish(msg)
        return jsonify({"ok": True, "count": len(clean)})

    def update_frame(self, frame):
        with self.lock:
            self.output_frame = frame
            self.frame_seq += 1
            self.frame_time = time.monotonic()

    def generate(self):
        # Only send NEW frames. The old loop re-encoded the last frame forever,
        # so a dead camera looked like a live (frozen) feed with no error.
        last_seq = -1
        while True:
            with self.lock:
                frame, seq = self.output_frame, self.frame_seq
            if frame is None or seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq
            # Encode outside the lock so a slow client cannot stall update_frame.
            flag, jpg = cv2.imencode(".jpg", frame)
            if flag:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')

    def video_feed(self):
        return Response(self.generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    # Also served on plain HTTP port 80, so the dashboard opens at
    # http://apex.local with no ":5000" to remember (apex.local is announced on
    # the hotspot by the host -- see HARDWARE.md). Best effort: binding 80 needs
    # root, which the robot's container has; anywhere else it is skipped.
    EXTRA_PORTS = (80,)

    def run(self):
        flask_thread = threading.Thread(target=lambda: self.app.run(
            host=self.host, port=self.port, debug=False, threaded=True, use_reloader=False
        ), daemon=True)
        flask_thread.start()

        from werkzeug.serving import make_server
        for port in self.EXTRA_PORTS:
            try:
                server = make_server(self.host, port, self.app, threaded=True)
            except (OSError, SystemExit) as e:
                print(f"[DASH] not also serving on port {port}: {e}")
                continue
            threading.Thread(target=server.serve_forever, daemon=True).start()
            print(f"[DASH] also serving on port {port}")
