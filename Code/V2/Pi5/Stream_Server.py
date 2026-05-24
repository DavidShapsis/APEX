import time
import cv2
import threading
from flask import Flask, Response, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class RobodogStreamer(Node):
    def __init__(self, host='0.0.0.0', port=5000):
        super().__init__('stream_server_node')
        
        # ROS 2 Publishers
        self.dir_pub = self.create_publisher(Int32, '/apex/navigation/cmd_dir', 10)
        self.nav_mode_pub = self.create_publisher(Bool, '/apex/navigation/nav_mode', 10)
        
        self.app = Flask(__name__)
        self.host = host
        self.port = port
        self.output_frame = None
        self.lock = threading.Lock()
        
        self.current_direction = 0 
        self.nav_mode = False  # Track state of autonomous navigation
        
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/set_direction', 'set_direction', self.set_direction, methods=['POST'])
        self.app.add_url_rule('/toggle_nav', 'toggle_nav', self.toggle_nav, methods=['POST'])

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
                .nav-btn {{ background:{nav_btn_color}; color:black; font-weight:bold; width:180px; padding:15px; margin:15px; border-radius:5px; cursor:pointer; border:none; }}
                input[type=range] {{ width: 80%; margin: 20px; }}
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
            </script>
        </head>
        <body>
            <h1>ROBODOG VISION</h1>
            <img src='/video_feed' style='width:90%; max-width:600px; border:2px solid #333;'>
            
            <h3>Direction: <span id="angleDisp">0°</span></h3>
            
            <div>
                <button class="btn" onclick="sendDir(-90)">LEFT</button>
                <button class="btn" onclick="sendDir(0)">FWD</button>
                <button class="btn" onclick="sendDir(90)">RIGHT</button>
                <button class="btn" onclick="sendDir(180)">BACK</button>
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