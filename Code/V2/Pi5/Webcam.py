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
            print(f"Error: Could not open webcam at index {device_index}")
            self.running = False
        else:
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