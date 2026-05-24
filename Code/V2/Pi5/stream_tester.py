#!/usr/bin/env python3
import time
import threading
import sys
import os
import socket
import cv2  # Explicitly importing to access backend flags

# --- 1. Path & Dependency Resolution ---
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    import rclpy
    from Webcam import USBWebcam
    from Stream_Server import RobodogStreamer
    print("[INIT] Dependencies imported successfully.")
except ImportError as e:
    print(f"[ERROR] Import Error: {e}")
    sys.exit(1)

# --- 2. Camera Worker Loop ---
def camera_loop(cam, streamer, stop_event):
    """Continuously captures frames from the webcam and updates the stream server."""
    print("[CAMERA] Video capture worker thread started.")
    while not stop_event.is_set():
        try:
            frame = cam.get_frame()
            if frame is not None:
                streamer.update_frame(frame)
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[CAMERA ERROR] Exception in capture loop: {e}")
            break
        time.sleep(0.03)
    print("[CAMERA] Video capture worker thread stopped.")

# --- 3. Self-Testing Verification Engine ---
def verify_network_socket(port=8080):
    """Programmatically checks if the streaming server port opened successfully."""
    time.sleep(1.5)
    print(f"[TEST] Verifying local network binding on port {port}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    
    result = sock.connect_ex(("127.0.0.1", port))
    sock.close()
    
    if result == 0:
        print("\n" + "="*60)
        print(f"✅ SUCCESS: Stream server is verified up and listening on port {port}!")
        print("1. Connect your device to the Robodog Hotspot Wi-Fi network.")
        print("2. Open a browser or VLC media player.")
        print("3. Navigate to your Pi 5 hotspot gateway IP (e.g., http://192.168.4.1:8080)")
        print("="*60 + "\n")
    else:
        print("\n" + "!"*60)
        print(f"❌ WARNING: Port {port} did not respond locally.")
        print("Check if Stream_Server.py uses a different port or is blocking on binding.")
        print("!"*60 + "\n")

# --- 4. Main Executive Flow ---
def main():
    STREAM_PORT = 8080 
    stop_event = threading.Event()
    
    if not rclpy.ok():
        print("[INIT] Initializing rclpy context...")
        rclpy.init()

    cam = None
    
    # Scan a wider spectrum including common alternative hardware indexes
    # Target index 23 directly based on our timeout feedback
    scan_indices = [0, 1, 23, 22] 
    
    for index in scan_indices:
        print(f"[INIT] Trying USB Webcam at index {index}...")
        try:
            test_cam = USBWebcam(device_index=index)
            
            # Explicit override to apply standard compatibility formats
            if hasattr(test_cam, 'cap'):
                test_cam.cap.release()
                
                # Dynamic index assignment with explicit V4L2 backend support
                test_cam.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
                
                # Force MJPEG pixel format codec (FourCC)
                test_cam.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                
                # Downscale resolution to ensure standard bus safety limits
                test_cam.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                test_cam.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            if hasattr(test_cam, 'cap') and test_cam.cap.isOpened():
                print(f"[INIT] Probing video stream from index {index}...")
                ret, frame_check = test_cam.cap.read()
                if ret and frame_check is not None:
                    cam = test_cam
                    print(f"[INIT] Successfully locked and verified frame capture on index {index}!")
                    break
                else:
                    print(f"[DEBUG] Index {index} opened but failed to read a frame matrix.")
                    test_cam.release()
            else:
                test_cam.release()
        except Exception as e:
            print(f"[DEBUG] Index {index} failed init step: {e}")
            pass

    if cam is None:
        print("[CRITICAL] Could not open a webcam at any probed indices.")
        print("Try running permissions validation: 'v4l2-ctl --device=/dev/video0 --all'")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    
    print("[INIT] Initializing Robodog Stream Server node...")
    streamer = RobodogStreamer()
    
    print("[NETWORK] Spinning up socket listeners...")
    streamer.run()
    
    cam_thread = threading.Thread(
        target=camera_loop, 
        args=(cam, streamer, stop_event), 
        daemon=True
    )
    cam_thread.start()
    
    test_thread = threading.Thread(
        target=verify_network_socket,
        args=(STREAM_PORT,),
        daemon=True
    )
    test_thread.start()
    
    print("[SYSTEM] All streaming sub-systems executing. Press Ctrl+C to terminate.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Intercepted exit command. Cleaning up allocations...")
    finally:
        stop_event.set()
        cam_thread.join(timeout=2.0)
        cam.release()
        
        if rclpy.ok():
            print("[SHUTDOWN] Destroying ROS 2 streaming node context...")
            streamer.destroy_node()
            rclpy.shutdown()
        print("[SHUTDOWN] Hardware resources completely released.")

if __name__ == "__main__":
    main()