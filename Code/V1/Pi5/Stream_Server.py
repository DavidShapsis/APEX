import cv2
import threading
from flask import Flask, Response, request

class RobodogStreamer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.app = Flask(__name__)
        self.host = host
        self.port = port
        self.output_frame = None
        self.lock = threading.Lock()
        
        # This will store the current direction to be read by the main script
        self.current_direction = 0 
        
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/set_direction', 'set_direction', self.set_direction, methods=['POST'])

    def index(self):
        return """
        <html>
        <head>
            <title>Robodog Mission Control</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body { background:#1a1a1a; color:#00ff00; text-align:center; font-family:sans-serif; }
                .btn { background:#333; color:white; border:1px solid #555; padding:15px; margin:5px; width:80px; border-radius:5px; cursor:pointer; }
                .btn:active { background:#00ff00; color:black; }
                input[type=range] { width: 80%; margin: 20px; }
            </style>
            <script>
                function sendDir(val) {
                    fetch('/set_direction', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                        body: 'angle=' + val
                    });
                    document.getElementById('angleDisp').innerText = val + '°';
                }
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
        </body>
        </html>
        """

    def set_direction(self):
        self.current_direction = int(request.form.get('angle', 0))
        return "OK"

    def update_frame(self, frame):
        with self.lock:
            self.output_frame = frame.copy()

    def generate(self):
        while True:
            with self.lock:
                if self.output_frame is None: continue
                (flag, encodedImage) = cv2.imencode(".jpg", self.output_frame)
                if not flag: continue
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

    def video_feed(self):
        return Response(self.generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def run(self):
        t = threading.Thread(target=lambda: self.app.run(host=self.host, port=self.port, debug=False, threaded=True, use_reloader=False))
        t.daemon = True
        t.start()