import time
import cv2
import threading
from flask import Flask, Response, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Bool

LEG_NAMES = ['FL', 'FR', 'RR', 'RL']

class RobodogStreamer(Node):
    def __init__(self, host='0.0.0.0', port=5000):
        super().__init__('stream_server_node')

        # ROS 2 Publishers
        self.dir_pub = self.create_publisher(Int32, '/apex/navigation/cmd_dir', 10)
        self.nav_mode_pub = self.create_publisher(Bool, '/apex/navigation/nav_mode', 10)
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

        # Updated by homing_status_callback from pi5_main.py's periodic publish.
        # Everything starts false/0 -- matches the real state at boot, since
        # nothing is homed until the operator does it from this UI.
        self.homing_status = {
            'homed': [0, 0, 0, 0], 'standing': 0, 'walking': 0,
            'deactivated': [0, 0, 0, 0],
        }

        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/set_direction', 'set_direction', self.set_direction, methods=['POST'])
        self.app.add_url_rule('/toggle_nav', 'toggle_nav', self.toggle_nav, methods=['POST'])
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
            self.homing_status = {
                'homed': data[0:4],
                'standing': data[4],
                'walking': data[5],
                'deactivated': data[6:10],
            }

    def index(self):
        # Dynamic button styling based on initial state
        nav_btn_color = "#ff0000" if not self.nav_mode else "#00ff00"
        nav_text = "NAV MODE: OFF" if not self.nav_mode else "NAV MODE: ON"

        return f"""
        <html>
        <head>
            <title>Robodog Mission Control</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body {{ background:#1a1a1a; color:#00ff00; text-align:center; font-family:sans-serif; }}
                .btn {{ background:#333; color:white; border:1px solid #555; padding:15px; margin:5px; width:80px; border-radius:5px; cursor:pointer; }}
                .btn:active {{ background:#00ff00; color:black; }}
                .btn:disabled {{ background:#222; color:#666; cursor:not-allowed; }}
                .nav-btn {{ background:{nav_btn_color}; color:black; font-weight:bold; width:180px; padding:15px; margin:15px; border-radius:5px; cursor:pointer; border:none; }}
                input[type=range] {{ width: 80%; margin: 20px; }}
                .home-section {{ border:1px solid #333; border-radius:8px; padding:10px; margin:15px auto; max-width:420px; }}
                .home-btn {{ background:#333; color:white; border:1px solid #555; padding:12px; margin:5px; width:70px; border-radius:5px; cursor:pointer; }}
                .home-btn.homed {{ background:#0a5; border-color:#0f6; }}
                .home-btn:disabled {{ cursor:not-allowed; }}
                .big-btn {{ padding:15px 25px; margin:8px; border-radius:6px; border:none; cursor:pointer; font-weight:bold; }}
                .big-btn:disabled {{ background:#333; color:#777; cursor:not-allowed; }}
                #warnBanner {{ background:#c00; color:white; padding:10px; font-weight:bold; display:none; }}
                .debug-section {{ border:1px solid #444; border-radius:8px; padding:10px; margin:15px auto; max-width:420px; }}
                .deact-btn {{ background:#333; color:white; border:1px solid #555; padding:10px; margin:5px; width:90px; border-radius:5px; cursor:pointer; }}
                .deact-btn.off {{ background:#a50; border-color:#f80; }}
            </style>
            <script>
                function sendDir(val) {{
                    fetch('/set_direction', {{"method": 'POST', "headers": {{'Content-Type': 'application/x-www-form-urlencoded'}}, "body": 'angle=' + val}});
                    document.getElementById('angleDisp').innerText = val + '°';
                }}
                function toggleNav() {{
                    fetch('/toggle_nav', {{"method": 'POST'}})
                    .then(response => response.json())
                    .then(data => {{
                        var btn = document.getElementById('navBtn');
                        if(data.nav_mode) {{
                            btn.style.background = '#00ff00';
                            btn.innerText = 'NAV MODE: ON';
                        }} else {{
                            btn.style.background = '#ff0000';
                            btn.innerText = 'NAV MODE: OFF';
                        }}
                    }});
                }}
                function homeLeg(id) {{
                    fetch('/home_leg', {{"method": 'POST', "headers": {{'Content-Type': 'application/x-www-form-urlencoded'}}, "body": 'leg=' + id}});
                }}
                function doStand() {{
                    fetch('/stand', {{"method": 'POST'}}).then(r => r.json()).then(d => {{
                        if (!d.ok) alert(d.error || 'Cannot stand yet.');
                    }});
                }}
                function doGo() {{
                    fetch('/go', {{"method": 'POST'}}).then(r => r.json()).then(d => {{
                        if (!d.ok) alert(d.error || 'Cannot go yet.');
                    }});
                }}
                function doStop() {{
                    fetch('/stop', {{"method": 'POST'}});
                }}
                function toggleLeg(id, currentlyDeactivated) {{
                    const url = currentlyDeactivated ? '/reactivate_leg' : '/deactivate_leg';
                    fetch(url, {{"method": 'POST', "headers": {{'Content-Type': 'application/x-www-form-urlencoded'}}, "body": 'leg=' + id}});
                }}
                const LEG_NAMES = ['FL', 'FR', 'RR', 'RL'];
                function refreshStatus() {{
                    fetch('/status').then(r => r.json()).then(s => {{
                        const allHomed = s.homed.every(v => v === 1);
                        LEG_NAMES.forEach((name, i) => {{
                            const btn = document.getElementById('home_' + name);
                            btn.classList.toggle('homed', s.homed[i] === 1);
                            btn.innerText = (s.homed[i] === 1 ? '✓ ' : '') + 'Home ' + name;

                            const deactBtn = document.getElementById('deact_' + name);
                            const isOff = s.deactivated[i] === 1;
                            deactBtn.classList.toggle('off', isOff);
                            deactBtn.innerText = (isOff ? 'Reactivate ' : 'Deactivate ') + name;
                            deactBtn.onclick = () => toggleLeg(i, isOff);
                        }});
                        document.getElementById('standBtn').disabled = !allHomed;
                        document.getElementById('goBtn').disabled = !(allHomed && s.standing === 1);
                        document.getElementById('goBtn').innerText = s.walking === 1 ? 'WALKING' : 'GO';
                        if (s.walking === 1) document.getElementById('goBtn').disabled = true;
                        document.getElementById('warnBanner').style.display = allHomed ? 'none' : 'block';
                    }}).catch(() => {{}});
                }}
                setInterval(refreshStatus, 1000);
                window.onload = refreshStatus;
            </script>
        </head>
        <body>
            <h1>ROBODOG VISION</h1>
            <img src='/video_feed' style='width:90%; max-width:600px; border:2px solid #333;'>

            <div id="warnBanner">⚠ Not every leg is homed — Stand and Go are disabled until you home all four.</div>

            <div class="home-section">
                <h3>Homing</h3>
                <p>Position each leg by hand (hip roll 90°, knee locked 180°, hip pitch 0°), then press its button.</p>
                <button id="home_FL" class="home-btn" onclick="homeLeg(0)">Home FL</button>
                <button id="home_FR" class="home-btn" onclick="homeLeg(1)">Home FR</button>
                <button id="home_RR" class="home-btn" onclick="homeLeg(2)">Home RR</button>
                <button id="home_RL" class="home-btn" onclick="homeLeg(3)">Home RL</button>
                <br>
                <button id="standBtn" class="big-btn" style="background:#08c;color:white;" onclick="doStand()" disabled>STAND</button>
                <button id="goBtn" class="big-btn" style="background:#0a5;color:white;" onclick="doGo()" disabled>GO</button>
                <button id="stopBtn" class="big-btn" style="background:#c00;color:white;" onclick="doStop()">STOP</button>
                <p style="font-size:0.85em;color:#999;">STOP cuts motor power (no holding torque) but keeps encoder
                tracking, so re-homing isn't needed to resume. It does NOT lower the robot first — only use it once
                the robot is off the ground or supported, not while it's standing/walking on its own legs.</p>
            </div>

            <div class="debug-section">
                <h3>Debug: per-leg deactivation</h3>
                <p style="font-size:0.85em;color:#999;">Testing only. A deactivated leg is still fully computed every
                gait step, it just receives no wire signal, so it holds its last position while the other three keep
                walking normally.</p>
                <button id="deact_FL" class="deact-btn" onclick="toggleLeg(0, false)">Deactivate FL</button>
                <button id="deact_FR" class="deact-btn" onclick="toggleLeg(1, false)">Deactivate FR</button>
                <button id="deact_RR" class="deact-btn" onclick="toggleLeg(2, false)">Deactivate RR</button>
                <button id="deact_RL" class="deact-btn" onclick="toggleLeg(3, false)">Deactivate RL</button>
            </div>

            <h3>Direction: <span id="angleDisp">0°</span></h3>

            <div>
                <button class="btn" onclick="sendDir(-90)">LEFT</button>
                <button class="btn" onclick="sendDir(0)">FWD</button>
                <button class="btn" onclick="sendDir(90)">RIGHT</button>
            </div>

            <input type="range" min="-180" max="180" value="0" oninput="sendDir(this.value)">
            <br>
            <button id="navBtn" class="nav-btn" onclick="toggleNav()">{nav_text}</button>
        </body>
        </html>
        """

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
