import unittest
import sys
import os

# Add the workspace directory and the typings directory to sys.path to resolve mocks on the host
sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))
sys.path.insert(0, os.path.join(sys.path[0], 'typings'))

import board
import motor
import code

class TestFirmwareSmoke(unittest.TestCase):

    def test_pin_allocations(self):
        """Verify that all hardware pin configurations are distinct to avoid hardware conflicts."""
        # Motor 1 pins
        m1_pins = {board.GP15, board.GP16, board.GP17, board.GP18}
        self.assertEqual(len(m1_pins), 4, "Motor 1 must use 4 unique pins")

        # Motor 2 pins
        m2_pins = {board.GP19, board.GP20, board.GP21, board.GP22}
        self.assertEqual(len(m2_pins), 4, "Motor 2 must use 4 unique pins")

        # Light sensors
        sensor_pins = {board.GP26, board.GP27}
        self.assertEqual(len(sensor_pins), 2, "Sensors must use 2 unique pins")

        # Ensure no overlap
        all_pins = m1_pins | m2_pins | sensor_pins
        self.assertEqual(len(all_pins), 10, "GPIO pins for motor 1, motor 2, and sensors must not overlap")

    def test_rpm_constraints(self):
        """Verify RPM settings are safe and logical."""
        self.assertGreater(code.MIN_RPM, 0, "MIN_RPM must be positive")
        self.assertGreater(code.MAX_RPM, code.MIN_RPM, "MAX_RPM must be greater than MIN_RPM")
        self.assertLessEqual(code.MAX_RPM, 20.0, "MAX_RPM should not exceed 20.0 for safety limits")

    def test_calibration_steps(self):
        """Verify calibration rotation is positive."""
        self.assertGreater(code.CALIBRATION_STEPS, 0, "Calibration steps must be positive")

    def test_stepper_motor_logic(self):
        """Verify stepper motor interface, stepping sequence, and RPM-to-interval conversion."""
        # Instantiate test motor
        test_motor = motor.StepperMotor(
            board.GP15, board.GP16, board.GP17, board.GP18,
            rpm=10, steps_per_rev=2048
        )
        
        # Test RPM mapping to step interval
        # 60 / (10 * 2048) = 0.0029296875 seconds
        self.assertAlmostEqual(test_motor.step_interval, 60 / (10 * 2048), places=6)
        
        # Test RPM updating
        test_motor.set_rpm(15)
        self.assertAlmostEqual(test_motor.step_interval, 60 / (15 * 2048), places=6)

        # Test movement targeting
        self.assertEqual(test_motor.position, 0)
        self.assertEqual(test_motor.target, 0)
        
        test_motor.move(100)
        self.assertEqual(test_motor.target, 100)
        self.assertTrue(test_motor.busy)

        test_motor.move_to(50)
        self.assertEqual(test_motor.target, 50)

        test_motor.stop()
        self.assertEqual(test_motor.target, test_motor.position)
        self.assertFalse(test_motor.run_forever)

        # Test step sequence indices and FULLSTEP output patterns
        initial_seq = test_motor.seq_index
        
        # Step forward (direction = 1)
        test_motor._apply_step(1)
        self.assertEqual(test_motor.seq_index, (initial_seq + 1) % 4)
        self.assertEqual(test_motor.position, 1)
        
        # Step backward (direction = -1)
        test_motor._apply_step(-1)
        self.assertEqual(test_motor.seq_index, initial_seq)
        self.assertEqual(test_motor.position, 0)

    def test_update_motor_behaviors_positive_polarity(self):
        """Test control loop decision math and speed scaling with LIGHT_POLARITY = 1 (more light = higher reading)."""
        # Save original states to restore later
        orig_polarity = code.LIGHT_POLARITY
        orig_min_l, orig_max_l, orig_thresh_l = code.min_left, code.max_left, code.threshold_left
        orig_min_r, orig_max_r, orig_thresh_r = code.min_right, code.max_right, code.threshold_right
        orig_min_rpm, orig_max_rpm = code.MIN_RPM, code.MAX_RPM

        try:
            # Set test configurations
            code.LIGHT_POLARITY = 1
            code.MIN_RPM, code.MAX_RPM = 5.0, 10.0
            code.min_left, code.max_left, code.threshold_left = 10000, 50000, 18000  # range 40000, thresh = 20% above min
            code.min_right, code.max_right, code.threshold_right = 20000, 60000, 28000

            # 1. Both below threshold -> Motors should stop
            rpm1, rpm2 = code.update_motor_behaviors(15000, 25000)
            self.assertEqual(rpm1, 0.0)
            self.assertEqual(rpm2, 0.0)
            self.assertFalse(code.motor1.run_forever)
            self.assertFalse(code.motor2.run_forever)

            # 2. Left sensor above threshold -> Motor 1 should move, Motor 2 stop
            # Left value 34000 is halfway between threshold (18000) and max (50000)
            # Math: 5.0 + (10.0 - 5.0) * (34000 - 18000) / (50000 - 18000) = 5.0 + 5.0 * 16000 / 32000 = 7.5 RPM
            rpm1, rpm2 = code.update_motor_behaviors(34000, 25000)
            self.assertAlmostEqual(rpm1, 7.5, places=2)
            self.assertEqual(rpm2, 0.0)
            self.assertTrue(code.motor1.run_forever)
            self.assertFalse(code.motor2.run_forever)

            # 3. Right sensor above threshold -> Motor 1 stop, Motor 2 move
            # Right value 60000 is at max -> Should scale to MAX_RPM (10.0)
            rpm1, rpm2 = code.update_motor_behaviors(15000, 60000)
            self.assertEqual(rpm1, 0.0)
            self.assertAlmostEqual(rpm2, 10.0, places=2)
            self.assertFalse(code.motor1.run_forever)
            self.assertTrue(code.motor2.run_forever)

        finally:
            # Restore original values
            code.LIGHT_POLARITY = orig_polarity
            code.MIN_RPM, code.MAX_RPM = orig_min_rpm, orig_max_rpm
            code.min_left, code.max_left, code.threshold_left = orig_min_l, orig_max_l, orig_thresh_l
            code.min_right, code.max_right, code.threshold_right = orig_min_r, orig_max_r, orig_thresh_r

    def test_update_motor_behaviors_negative_polarity(self):
        """Test control loop decision math with LIGHT_POLARITY = -1 (more light = lower reading)."""
        # Save original states
        orig_polarity = code.LIGHT_POLARITY
        orig_min_l, orig_max_l, orig_thresh_l = code.min_left, code.max_left, code.threshold_left
        orig_min_rpm, orig_max_rpm = code.MIN_RPM, code.MAX_RPM

        try:
            # Set test configurations
            code.LIGHT_POLARITY = -1
            code.MIN_RPM, code.MAX_RPM = 5.0, 10.0
            # range = 40000, thresh = 20% below max (50000 - 8000 = 42000)
            code.min_left, code.max_left, code.threshold_left = 10000, 50000, 42000

            # 1. Left value 45000 is above threshold -> No light detected (RPM = 0)
            rpm1, _ = code.update_motor_behaviors(45000, 50000)
            self.assertEqual(rpm1, 0.0)

            # 2. Left value 26000 is below threshold -> Light detected
            # Math: 5.0 + (10.0 - 5.0) * (42000 - 26000) / (42000 - 10000) = 5.0 + 5.0 * 16000 / 32000 = 7.5 RPM
            rpm1, _ = code.update_motor_behaviors(26000, 50000)
            self.assertAlmostEqual(rpm1, 7.5, places=2)
            self.assertTrue(code.motor1.run_forever)

        finally:
            code.LIGHT_POLARITY = orig_polarity
            code.MIN_RPM, code.MAX_RPM = orig_min_rpm, orig_max_rpm
            code.min_left, code.max_left, code.threshold_left = orig_min_l, orig_max_l, orig_thresh_l

if __name__ == '__main__':
    unittest.main()
