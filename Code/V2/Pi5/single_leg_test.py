"""
single_leg_test.py
------------------
Single-leg test harness for Apex robot dog.
- One Pico connected on a single UART port
- Webcam stream to browser at http://<pi-ip>:5000
- Web UI for direction control and gait tuning
- No IMU, GPS, audio, or ROS dependencies
"""
 
import threading
import math
import time
import serial
import struct
import cv2
from flask import Flask, Response, request, jsonify
 
from InverseKinematics.ik_and_gait import InverseKinematics, GaitPath, GaitIK, RecoveryPath
 
# ─────────────────────────────────────────────
# CONFIG — edit these to match your hardware
# ─────────────────────────────────────────────
PICO_PORT    = '/dev/ttyAMA0'       # UART port for the single test leg
BAUD_RATE    = 115200
CAMERA_INDEX = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_Camera_SN0001-video-index0"
FLASK_PORT   = 5000
 
# Leg geometry (cm) — same as production
IK_SEGMENTS = {'a': 9.65, 'b': 26.84, 'c': 24.37}
 
# Gait defaults
DEFAULT_HEIGHT_Z   = 36.0
DEFAULT_STRIDE_LEN = 10.0
DEFAULT_HEIGHT1    = 5.0
DEFAULT_HEIGHT2    = 2.5
# ─────────────────────────────────────────────
 
 
class SingleLegController:
    def __init__(self):
        self.ser = None
        self._connect_serial()
 
        self.ik_engine      = InverseKinematics(IK_SEGMENTS)
        self.path_gen       = GaitPath()
        self.recovery_engine = RecoveryPath(self.ik_engine)
 
        self.path_gen.update_params(
            center_stride_y=0.0,
            center_height_z=DEFAULT_HEIGHT_Z,
            length=DEFAULT_STRIDE_LEN,
            height1=DEFAULT_HEIGHT1,
            height2=DEFAULT_HEIGHT2,
            direction_angle=0
        )
        self.gait_processor = GaitIK(self.ik_engine, self.path_gen.gait_xy_path)
        self.all_angles     = self.gait_processor.get_gait_ik()
 
        self.end_marker   = b'\xFF' * 16
        self.serial_lock  = threading.Lock()
        self.gait_queue   = None        # pending gait array for the worker
        self.recovery_job = None        # (recovery_gait,) pending recovery
        self.is_running   = True
        self.is_recovery  = False
 
        # Live tuning params (readable from web UI)
        self.direction     = 0          # degrees, -180..180
        self.stride_length = DEFAULT_STRIDE_LEN
        self.height_z      = DEFAULT_HEIGHT_Z
        self.lateral_offset = 0.0
 
        self.worker = threading.Thread(target=self._serial_worker, daemon=True)
        self.worker.start()
 
    # ── Serial setup ──────────────────────────────────────────────────────────
 
    def _connect_serial(self):
        try:
            self.ser = serial.Serial(PICO_PORT, baudrate=BAUD_RATE, timeout=0.1)
            print(f"[SERIAL] Connected to Pico on {PICO_PORT}")
        except Exception as e:
            print(f"[SERIAL] Failed to open {PICO_PORT}: {e}")
            self.ser = None
 
    # ── Gait generation ───────────────────────────────────────────────────────
 
    def regenerate_gait(self, direction=None, stride_length=None, height_z=None, lateral_offset=None):
        """Rebuild the gait path from current or supplied params and queue it."""
        if direction     is not None: self.direction      = direction
        if stride_length is not None: self.stride_length  = stride_length
        if height_z      is not None: self.height_z       = height_z
        if lateral_offset is not None: self.lateral_offset = lateral_offset
 
        direction_rad       = math.radians(self.direction)
        longitudinal_stride = self.stride_length * math.cos(direction_rad)
        lateral_stride      = self.stride_length * 0.5 * math.sin(direction_rad)
 
        self.path_gen.update_params(
            center_stride_y=0.0,
            center_height_z=self.height_z,
            length=longitudinal_stride,
            height1=DEFAULT_HEIGHT1,
            height2=DEFAULT_HEIGHT2,
            direction_angle=0
        )
        self.gait_processor = GaitIK(
            self.ik_engine,
            self.path_gen.gait_xy_path,
            lateral_roll_offset=self.lateral_offset + lateral_stride
        )
        new_angles = self.gait_processor.get_gait_ik()
 
        with self.serial_lock:
            self.gait_queue = new_angles
 
        return new_angles
 
    def queue_recovery(self, abort_line):
        """Parse ABORTED message from Pico and queue a recovery path."""
        try:
            parts = abort_line.split(',')
            curr_roll  = float(parts[1])
            curr_pitch = float(parts[2])
            curr_knee  = float(parts[3])
 
            start_x, start_y, start_z = self.ik_engine.calculate_fk(curr_roll, curr_pitch, curr_knee)
            recovery_gait = self.recovery_engine.get_recovery_gait(start_x, start_y, start_z)
 
            with self.serial_lock:
                self.is_recovery  = True
                self.recovery_job = recovery_gait
                self.gait_queue   = None
            print(f"[RECOVERY] Queued recovery from ({start_x:.1f}, {start_y:.1f}, {start_z:.1f})")
        except Exception as e:
            print(f"[RECOVERY] Parse error: {e}")
 
    # ── Background serial worker ───────────────────────────────────────────────
 
    def _serial_worker(self):
        local_gait = None
 
        while self.is_running:
            # --- Grab state under lock, release immediately ---
            pending_recovery = None
            with self.serial_lock:
                if self.is_recovery and self.recovery_job is not None:
                    pending_recovery  = self.recovery_job
                    self.recovery_job = None
                elif self.gait_queue is not None:
                    local_gait      = self.gait_queue
                    self.gait_queue = None
 
            # --- Recovery transmission (lock NOT held) ---
            if pending_recovery is not None:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.reset_output_buffer()
                        self.ser.write(b'\xAA\xAA')
                        for step in pending_recovery:
                            packed = struct.pack('ffff', float(step[0]), float(step[1]),
                                                         float(step[2]), float(step[3]))
                            self.ser.write(packed)
                            time.sleep(0.01)
                        self.ser.write(self.end_marker)
                        print("[RECOVERY] Transmission complete")
                    except Exception as e:
                        print(f"[RECOVERY] Serial error: {e}")
 
                with self.serial_lock:
                    self.is_recovery = False
                local_gait = None
                continue
 
            if local_gait is None:
                time.sleep(0.005)
                continue
 
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
 
            # --- Normal gait transmission ---
            try:
                self.ser.reset_output_buffer()
                self.ser.write(b'\xAA\xAA')
            except Exception as e:
                print(f"[SERIAL] Write error: {e}")
                continue
 
            aborted = False
            for step in local_gait:
                with self.serial_lock:
                    if self.is_recovery:
                        aborted = True
                        break
                packed = struct.pack('ffff', float(step[0]), float(step[1]),
                                             float(step[2]), float(step[3]))
                try:
                    self.ser.write(packed)
                except Exception as e:
                    print(f"[SERIAL] Step write error: {e}")
                time.sleep(0.001)
 
            if aborted:
                local_gait = None
                continue
 
            try:
                self.ser.write(self.end_marker)
            except Exception as e:
                print(f"[SERIAL] End marker error: {e}")
 
    # ── Inbound serial reader (runs in its own thread) ─────────────────────────
 
    def _serial_reader(self):
        """Reads ABORTED messages back from the Pico."""
        while self.is_running:
            if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("ABORTED"):
                        print(f"[ABORT] Received: {line}")
                        self.queue_recovery(line)
                except Exception as e:
                    print(f"[SERIAL] Read error: {e}")
            time.sleep(0.005)
 
    def start_reader(self):
        threading.Thread(target=self._serial_reader, daemon=True).start()
 
    def close(self):
        self.is_running = False
        self.worker.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[SERIAL] Closed")
 
 
# ─────────────────────────────────────────────
# Camera
# ─────────────────────────────────────────────
 
class Camera:
    def __init__(self, device):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame  = None
        self.lock   = threading.Lock()
        self.active = self.cap.isOpened()
        if not self.active:
            print(f"[CAM] Could not open camera: {device}")
        else:
            print(f"[CAM] Camera online: {device}")
 
    def update_loop(self):
        while True:
            if self.active:
                ret, frame = self.cap.read()
                if ret:
                    with self.lock:
                        self.frame = frame
            time.sleep(0.03)
 
    def get_jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            ok, buf = cv2.imencode('.jpg', self.frame)
            return bytearray(buf) if ok else None
 
    def release(self):
        self.cap.release()
 
 
# ─────────────────────────────────────────────
# Flask web server
# ─────────────────────────────────────────────
 
def make_app(controller: SingleLegController, camera: Camera) -> Flask:
    app = Flask(__name__)
 
    def gen_frames():
        while True:
            jpg = camera.get_jpeg()
            if jpg:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
            time.sleep(0.03)
 
    @app.route('/')
    def index():
        return """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Apex — Single Leg Test</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                * { box-sizing: border-box; margin: 0; padding: 0; }
                body {
                    background: #0d0d0d;
                    color: #e0e0e0;
                    font-family: 'Courier New', monospace;
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    padding: 20px;
                    gap: 16px;
                }
                h1 { color: #00ff88; letter-spacing: 4px; font-size: 1.2rem; margin-top: 8px; }
                h3 { color: #888; font-size: 0.75rem; letter-spacing: 2px; }
                img {
                    width: 90%;
                    max-width: 580px;
                    border: 1px solid #222;
                    border-radius: 4px;
                    background: #111;
                }
                .panel {
                    width: 90%;
                    max-width: 580px;
                    background: #111;
                    border: 1px solid #222;
                    border-radius: 6px;
                    padding: 16px;
                }
                .panel h3 { margin-bottom: 12px; }
                .dir-grid {
                    display: grid;
                    grid-template-columns: repeat(3, 1fr);
                    gap: 8px;
                    margin-bottom: 12px;
                }
                .btn {
                    background: #1a1a1a;
                    color: #ccc;
                    border: 1px solid #333;
                    padding: 12px 8px;
                    border-radius: 4px;
                    cursor: pointer;
                    font-family: inherit;
                    font-size: 0.8rem;
                    letter-spacing: 1px;
                    transition: background 0.1s, color 0.1s;
                }
                .btn:active, .btn.active {
                    background: #00ff88;
                    color: #000;
                    border-color: #00ff88;
                }
                .btn.stop { border-color: #ff4444; color: #ff4444; }
                .btn.stop:active { background: #ff4444; color: #000; }
                .spacer { visibility: hidden; }
                .slider-row {
                    display: flex;
                    align-items: center;
                    gap: 12px;
                    margin-bottom: 10px;
                }
                .slider-row label {
                    width: 130px;
                    font-size: 0.75rem;
                    color: #888;
                    flex-shrink: 0;
                }
                .slider-row input[type=range] { flex: 1; }
                .slider-row span {
                    width: 45px;
                    text-align: right;
                    font-size: 0.8rem;
                    color: #00ff88;
                }
                #status {
                    font-size: 0.7rem;
                    color: #555;
                    letter-spacing: 1px;
                    min-height: 18px;
                }
            </style>
        </head>
        <body>
            <h1>&#9632; APEX TEST RIG</h1>
            <p id="status">IDLE</p>
 
            <img src="/video_feed" alt="camera feed">
 
            <!-- Direction panel -->
            <div class="panel">
                <h3>DIRECTION</h3>
                <div class="dir-grid">
                    <div class="spacer"></div>
                    <button class="btn" onclick="sendDir(0)">FWD</button>
                    <div class="spacer"></div>
                    <button class="btn" onclick="sendDir(-90)">LEFT</button>
                    <button class="btn stop" onclick="sendDir(0); setStride(0)">STOP</button>
                    <button class="btn" onclick="sendDir(90)">RIGHT</button>
                    <div class="spacer"></div>
                    <button class="btn" onclick="sendDir(180)">BACK</button>
                    <div class="spacer"></div>
                </div>
                <div class="slider-row">
                    <label>ANGLE (deg)</label>
                    <input type="range" min="-180" max="180" value="0" step="5"
                           oninput="sendDir(parseInt(this.value))">
                    <span id="dirVal">0°</span>
                </div>
            </div>
 
            <!-- Tuning panel -->
            <div class="panel">
                <h3>GAIT TUNING</h3>
                <div class="slider-row">
                    <label>STRIDE (cm)</label>
                    <input type="range" min="0" max="20" value="10" step="0.5"
                           oninput="setStride(parseFloat(this.value))">
                    <span id="strideVal">10</span>
                </div>
                <div class="slider-row">
                    <label>HEIGHT (cm)</label>
                    <input type="range" min="25" max="45" value="36" step="0.5"
                           oninput="setHeight(parseFloat(this.value))">
                    <span id="heightVal">36</span>
                </div>
                <div class="slider-row">
                    <label>LATERAL (cm)</label>
                    <input type="range" min="-5" max="5" value="0" step="0.1"
                           oninput="setLateral(parseFloat(this.value))">
                    <span id="latVal">0</span>
                </div>
            </div>
 
            <script>
                let currentDir    = 0;
                let currentStride = 10;
                let currentHeight = 36;
                let currentLat    = 0;
                let debounce      = null;
 
                function status(msg) {
                    document.getElementById('status').innerText = msg;
                }
 
                function post(endpoint, params) {
                    clearTimeout(debounce);
                    debounce = setTimeout(() => {
                        fetch(endpoint, {
                            method: 'POST',
                            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                            body: new URLSearchParams(params).toString()
                        }).then(r => r.json()).then(d => status(d.msg || 'OK'));
                    }, 50);
                }
 
                function sendDir(val) {
                    currentDir = val;
                    document.getElementById('dirVal').innerText = val + '°';
                    post('/update', {dir: currentDir, stride: currentStride,
                                     height: currentHeight, lateral: currentLat});
                }
 
                function setStride(val) {
                    currentStride = val;
                    document.getElementById('strideVal').innerText = val.toFixed(1);
                    post('/update', {dir: currentDir, stride: currentStride,
                                     height: currentHeight, lateral: currentLat});
                }
 
                function setHeight(val) {
                    currentHeight = val;
                    document.getElementById('heightVal').innerText = val.toFixed(1);
                    post('/update', {dir: currentDir, stride: currentStride,
                                     height: currentHeight, lateral: currentLat});
                }
 
                function setLateral(val) {
                    currentLat = val;
                    document.getElementById('latVal').innerText = val.toFixed(1);
                    post('/update', {dir: currentDir, stride: currentStride,
                                     height: currentHeight, lateral: currentLat});
                }
            </script>
        </body>
        </html>
        """
 
    @app.route('/video_feed')
    def video_feed():
        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
 
    @app.route('/update', methods=['POST'])
    def update():
        try:
            direction      = float(request.form.get('dir',     0))
            stride_length  = float(request.form.get('stride',  DEFAULT_STRIDE_LEN))
            height_z       = float(request.form.get('height',  DEFAULT_HEIGHT_Z))
            lateral_offset = float(request.form.get('lateral', 0.0))
 
            controller.regenerate_gait(
                direction=direction,
                stride_length=stride_length,
                height_z=height_z,
                lateral_offset=lateral_offset
            )
            return jsonify({"msg": f"DIR:{direction:.0f}° STR:{stride_length:.1f} HT:{height_z:.1f}"})
        except Exception as e:
            return jsonify({"msg": f"ERROR: {e}"}), 400
 
    return app
 
 
# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────
 
def main():
    print("=" * 45)
    print("  APEX SINGLE-LEG TEST HARNESS")
    print("=" * 45)
 
    controller = SingleLegController()
    controller.start_reader()
 
    camera = Camera(CAMERA_INDEX)
    threading.Thread(target=camera.update_loop, daemon=True).start()
 
    # Send initial gait
    controller.regenerate_gait()
    print(f"[GAIT] Initial gait queued — {len(controller.all_angles)} steps")
    print(f"[WEB]  Control panel → http://<pi-ip>:{FLASK_PORT}/")
 
    app = make_app(controller, camera)
 
    try:
        app.run(host='0.0.0.0', port=FLASK_PORT, debug=False,
                threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[SHUTDOWN] Closing hardware...")
        controller.close()
        camera.release()
        print("[SHUTDOWN] Done.")
 
 
if __name__ == '__main__':
    main()