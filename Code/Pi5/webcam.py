import glob
import os

import cv2


def find_usb_camera():
    """/dev/videoN of the first USB camera's video node, or None.

    The Pi 5 registers ~17 /dev/video* nodes for its own ISP and decoder, so
    "video0" is not reliably the webcam, and /dev/v4l/by-id is made by udev on
    the host and does not exist inside the Docker container. sysfs is visible
    in both: keep nodes whose device sits on the USB bus, and of each camera's
    nodes take index 0 -- the picture; index 1 is its metadata node.
    """
    nodes = glob.glob("/sys/class/video4linux/video*")
    for node in sorted(nodes, key=lambda p: int(os.path.basename(p)[5:])):
        try:
            if "/usb" not in os.path.realpath(os.path.join(node, "device")):
                continue
            with open(os.path.join(node, "index")) as f:
                if f.read().strip() != "0":
                    continue
        except (OSError, ValueError):
            continue
        return "/dev/" + os.path.basename(node)
    return None


class USBWebcam:
    def __init__(self, device_index=None, width=640, height=480):
        """
        Initialize the webcam with explicit V4L2 and MJPEG formats for Pi 5.
        device_index None finds the USB camera automatically (find_usb_camera).
        """
        if device_index is None:
            device_index = find_usb_camera()
            if device_index is None:
                raise RuntimeError("no USB camera found")
        # Force the V4L2 backend driver directly
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        
        # Force the MJPEG pixel format codec 
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not self.cap.isOpened():
            # Raise, don't just flag: main() brings the camera up through
            # _bring_up(), which catches this, marks the camera subsystem down
            # (dashboard degrade banner) and skips camera_loop entirely. A
            # half-alive object that only ever returns None would instead read
            # as healthy while obstacle avoidance silently starves for frames.
            self.cap.release()
            raise RuntimeError(f"could not open webcam at {device_index}")
        print(f"Webcam successfully initialized at index {device_index}!")
        self.running = True

    def get_frame(self):
        """
        Returns the frame in BGR format (Standard OpenCV NumPy array).
        Returns None if frame capture fails.
        """
        if self.running:
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None

    def release(self):
        """Properly close the camera hardware."""
        self.cap.release()
        cv2.destroyAllWindows()