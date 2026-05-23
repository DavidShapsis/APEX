import cv2

class USBWebcam:
    def __init__(self, device_index=0, width=640, height=480):
        """
        Initialize the webcam.
        device_index: Usually 0 for the first USB cam.
        """
        self.cap = cv2.VideoCapture(device_index)
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not self.cap.isOpened():
            print(f"Error: Could not open webcam at index {device_index}")
            self.running = False
        else:
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