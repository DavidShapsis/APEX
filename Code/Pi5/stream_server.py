import time
import cv2
import threading
from flask import Flask, Response, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Bool

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

        # Updated by homing_status_callback from pi5_main.py's periodic publish.
        # Everything starts false/0 -- matches the real state at boot, since
        # nothing is homed until the operator does it from this UI.
        self.homing_status = {
            'homed': [0, 0, 0, 0], 'standing': 0, 'walking': 0,
            'deactivated': [0, 0, 0, 0],
            'avoid_available': 0, 'avoid_enabled': 0, 'avoid_state': 0,
            'avoid_steer': 0, 'avoid_stride': 100,
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
