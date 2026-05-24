import sys
import time
import unittest
from unittest.mock import MagicMock, patch

# --- 1. Universal Hardware Mocks ---
# These prevent scripts from crashing when looking for real I2C buses or serial ports
sys.modules['serial'] = MagicMock()
sys.modules['smbus2'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['power_monitor'] = MagicMock()
sys.modules['audio'] = MagicMock()
sys.modules['webcam'] = MagicMock()
sys.modules['stream_server'] = MagicMock()
sys.modules['navigation'] = MagicMock()
sys.modules['imu'] = MagicMock()

# Now we can safely import the targets without triggering physical hardware checks
import rclpy
from pi5_main import PiQuadrupedController, RobotState, main

class TestPi5QuadrupedSoftware(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context once for the test suite context."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context gracefully."""
        if rclpy.ok():
            rclpy.shutdown()

    @patch('serial.Serial')
    def test_controller_initialization(self, mock_serial):
        """Verifies that the controller node initializes, sets up kinematic engines, and creates global variables."""
        # Setup mock serial instances
        mock_instance = MagicMock()
        mock_serial.return_value = mock_instance
        
        # Instantiate node
        controller = PiQuadrupedController()
        
        # --- FIXED FOR PRODUCTION: The end_marker is a 16-byte protocol definition block (4 floats) ---
        self.assertTrue(hasattr(controller, 'end_marker'), "Missing 'end_marker' attribute in controller setup!")
        self.assertEqual(controller.end_marker, b'\xFF' * 16, "The end_marker is not configured to the 16-byte NaN block.")
        
        # Verify initial state engine allocation
        self.assertEqual(controller.current_state, RobotState.MANUAL)
        self.assertIsNotNone(controller.ik_engine)
        self.assertIsNotNone(controller.path_gen)
        
        # Clean up background threads spawned by init
        controller.close_hardware()

    @patch('serial.Serial')
    def test_recovery_state_transition_and_framing(self, mock_serial):
        """Verifies that handle_recovery locks state threads, parses metrics, and streams correct NaN termination blocks."""
        mock_port = MagicMock()
        mock_serial.return_value = mock_port
        
        controller = PiQuadrupedController()
        
        # Bypass forward kinematics lookup calculations for pure logic parsing verification
        # The internal calculation yields raw motor joint values [roll, pitch, knee]
        controller.ik_engine.calculate = MagicMock(return_value=[0.0, 0.0, 0.0])
        
        # --- FIXED FOR TRUNCATION CRASH: Provide exact 4-value elements containing [X, Y, Z, is_swing] ---
        # The background gait serial worker processes frames expecting a swing flag at index 3.
        valid_recovery_coordinates = [
            [0.0, 0.0, 30.0, False],
            [0.0, 0.0, 33.0, False],
            [0.0, 0.0, 36.0, False]
        ]
        controller.recovery_engine.get_recovery_gait = MagicMock(return_value=valid_recovery_coordinates)
        
        # Formulate a structured abort string matching what the Pico transmits on touchdown
        sample_abort_msg = "ABORTED,4.5,-2.1,12.8"
        
        # Execute recovery procedure
        controller.handle_recovery(sample_abort_msg, mock_port)
        
        # Allow the background worker thread a brief tick to process the step queue and clear the state
        timeout = 1.0
        start_time = time.time()
        while controller.current_state == RobotState.RECOVERY and (time.time() - start_time) < timeout:
            time.sleep(0.01)
            
        # Ensure recovery reverted back to previous state tracking automatically
        self.assertEqual(controller.current_state, RobotState.MANUAL, "The recovery routine failed to release the state machine back to MANUAL mode.")
        
        # Verify the binary stream protocol formatting matches exactly what we fixed
        # Expected: START marker, 16-byte float pack, then 16-byte NaN termination block
        mock_port.write.assert_any_call(b'\xAA\xAA')
        mock_port.write.assert_any_call(b'\xFF' * 16)
        
        controller.close_hardware()

if __name__ == '__main__':
    print("Executing standalone headless validation suite...")
    unittest.main()