import cv2

class USBWebcam:
    def __init__(self, device_index=0, width=640, height=480):
        """
        Initialize the webcam with explicit V4L2 and MJPEG formats for Pi 5.
        """
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