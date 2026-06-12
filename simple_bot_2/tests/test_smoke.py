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

    def test_fit_line_rmse_flat(self):
        """Verify that a perfect straight line yields 0.0 RMSE."""
        x = [1, 2, 3, 4, 5]
        y = [2, 4, 6, 8, 10]  # y = 2x
        rmse = code.fit_line_rmse(x, y)
        self.assertAlmostEqual(rmse, 0.0, places=6)

    def test_fit_line_rmse_noisy(self):
        """Verify that a noisy line yields a correct small non-zero RMSE."""
        x = [1, 2, 3, 4, 5]
        y = [2.1, 3.9, 6.2, 7.8, 10.1]
        rmse = code.fit_line_rmse(x, y)
        self.assertGreater(rmse, 0.0)
        self.assertLess(rmse, 0.2)

    def test_fit_line_rmse_curved(self):
        """Verify that a curved segment yields a significantly larger RMSE."""
        # Arc of a circle: y = 20 - sqrt(225 - x^2) for x in [-10, 0, 10]
        x = [-10, -5, 0, 5, 10]
        y = [8.82, 5.86, 5.0, 5.86, 8.82]
        rmse = code.fit_line_rmse(x, y)
        self.assertGreater(rmse, 0.5)

    def test_analyze_sweep_flat_vs_curved(self):
        """Test analyze_sweep outputs for simulated flat vs curved profiles."""
        # Simulated flat side (triangle normal): d = 30 / cos(theta)
        import math
        angles = list(range(-20, 22, 2))
        dists_flat = [30.0 / math.cos(math.radians(a)) for a in angles]
        
        score_flat = code.analyze_sweep(angles, dists_flat)
        self.assertIsNotNone(score_flat)
        self.assertLess(score_flat, 0.2)

        # Simulated circle: cylinder of R=15 at D=35 from sensor.
        # d^2 - 2*D*d*cos(theta) + D^2 - R^2 = 0
        # For D=35, R=15: d^2 - 70*d*cos(theta) + 1000 = 0
        # d = 35*cos(theta) - sqrt(1225*cos^2(theta) - 1000)
        dists_circle = []
        for a in angles:
            rad = math.radians(a)
            cos_a = math.cos(rad)
            d = 35.0 * cos_a - math.sqrt(1225.0 * cos_a**2 - 1000.0)
            dists_circle.append(d)

        score_circle = code.analyze_sweep(angles, dists_circle)
        self.assertIsNotNone(score_circle)
        self.assertGreater(score_circle, 0.2)
    def test_filter_sweep_spikes(self):
        """Verify that filter_sweep_spikes detects and removes sudden dips/peaks."""
        angles = [0, 2, 4, 6, 8]
        # Includes a sudden dip of 10.0 (normally ~30) and a sudden peak of 50.0
        dists = [30.0, 10.0, 31.0, 50.0, 30.0]
        
        clean_angles, clean_dists = code.filter_sweep_spikes(angles, dists)
        
        # Dip at index 1 and peak at index 3 should be removed
        self.assertEqual(len(clean_dists), 3)
        self.assertEqual(clean_dists, [30.0, 31.0, 30.0])
        self.assertEqual(clean_angles, [0, 4, 8])

if __name__ == '__main__':
    unittest.main()
