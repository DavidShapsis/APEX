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
        self.lock = threading.Lock()

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
        self.nav_mode = not self.nav_mode
        msg = Bool()
        msg.data = self.nav_mode
        self.nav_mode_pub.publish(msg)
        return jsonify({"nav_mode": self.nav_mode})

    def nav_control(self):
        """Route transport buttons. Body: form field action=start|pause|stop.
        Maps to nav_cmd 1/2/3 on the controller."""
        action = request.form.get('action', '')
        code = {'start': 1, 'pause': 2, 'stop': 3}.get(action)
        if code is None:
            return jsonify({"ok": False, "error": "action must be start, pause or stop"}), 400
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
        return jsonify(self.homing_status)

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

    def generate(self):
        while True:
            with self.lock:
                if self.output_frame is not None:
                    (flag, encodedImage) = cv2.imencode(".jpg", self.output_frame)
                    if flag:
                        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
            time.sleep(0.03)

    def video_feed(self):
        return Response(self.generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def run(self):
        flask_thread = threading.Thread(target=lambda: self.app.run(
            host=self.host, port=self.port, debug=False, threaded=True, use_reloader=False
        ), daemon=True)
        flask_thread.start()
