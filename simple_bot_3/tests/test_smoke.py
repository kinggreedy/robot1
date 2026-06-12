import unittest
import sys
import os
import math

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

        # Light sensors (GP26 and GP27 in code.py)
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

    def test_dft_frequency_response(self):
        """Verify that the DFT calculation correctly identifies 0.5 Hz signals and rejects DC/other frequencies."""
        # Setup local parameters mirroring code.py configuration
        FS = code.FS
        TARGET_FREQ = code.TARGET_FREQ
        N = code.N
        cos_table = code.cos_table
        sin_table = code.sin_table

        def compute_mag(buffer):
            mean = sum(buffer) / N
            ac = [x - mean for x in buffer]
            real = sum(x * c for x, c in zip(ac, cos_table))
            imag = sum(x * s for x, s in zip(ac, sin_table))
            return math.sqrt(real*real + imag*imag) / (N / 2)

        # Case 1: Pure DC signal (ambient light) -> Magnitude should be near 0
        dc_buffer = [32000.0] * N
        self.assertLess(compute_mag(dc_buffer), 1e-5)

        # Case 2: Pure 0.5 Hz sine wave of amplitude 1000.0 (Target frequency)
        # magnitude should be equal to the amplitude (1000.0)
        target_buffer = [1000.0 * math.sin(2.0 * math.pi * TARGET_FREQ * (n / FS)) for n in range(N)]
        mag = compute_mag(target_buffer)
        self.assertAlmostEqual(mag, 1000.0, places=1)

        # Case 3: 2.0 Hz sine wave (interfering frequency) -> Magnitude should be near 0
        other_buffer = [1000.0 * math.sin(2.0 * math.pi * 2.0 * (n / FS)) for n in range(N)]
        self.assertLess(compute_mag(other_buffer), 1.0)

    def test_dft_neopixel_crosstalk_rejection(self):
        """Verify the filter rejects 1.0 Hz signals (representing NeoPixel search blinking crosstalk)."""
        FS = code.FS
        N = code.N
        cos_table = code.cos_table
        sin_table = code.sin_table

        # Simulate a 1.0 Hz signal (mains or NeoPixel blinking at 1.0 Hz in SEARCHING state)
        neopixel_buffer = [32000.0 + 1000.0 * math.sin(2.0 * math.pi * 1.0 * (n / FS)) for n in range(N)]

        mean = sum(neopixel_buffer) / N
        ac = [x - mean for x in neopixel_buffer]
        real = sum(x * c for x, c in zip(ac, cos_table))
        imag = sum(x * s for x, s in zip(ac, sin_table))
        mag = math.sqrt(real*real + imag*imag) / (N / 2)

        # Teammate's standard DFT is perfectly orthogonal to 1.0 Hz over 40 samples (Bin 2 vs Bin 1), yielding 0.000.
        # (The differencing filter fails this test as it leaks 103.58 into the 0.5Hz bin due to window truncation).
        self.assertLess(mag, 1.0, "Filter must reject 1.0 Hz status light blinking crosstalk")

    def test_dft_motor_noise_rejection(self):
        """Verify 3.0 RPM motor noise is rejected, while 3.5 RPM noise aliases and leaks above threshold."""
        FS = code.FS
        N = code.N
        cos_table = code.cos_table
        sin_table = code.sin_table
        
        # 3.5 RPM motor noise aliases to ~0.533 Hz (fundamental step frequency of 119.47 Hz)
        # 3.5 RPM * 2048 steps/rev / 60s = 119.467 Hz. Sampled at 20 Hz, aliases to |119.467 - 6*20| = 0.533 Hz.
        f_noise_35 = 0.533
        buffer_35 = [32000.0 + 1000.0 * math.sin(2.0 * math.pi * f_noise_35 * (n / FS)) for n in range(N)]
        
        mean_35 = sum(buffer_35) / N
        ac_35 = [x - mean_35 for x in buffer_35]
        real_35 = sum(x * c for x, c in zip(ac_35, cos_table))
        imag_35 = sum(x * s for x, s in zip(ac_35, sin_table))
        mag_35 = math.sqrt(real_35*real_35 + imag_35*imag_35) / (N / 2)
        
        # 3.0 RPM motor noise aliases to ~2.4 Hz (fundamental step frequency of 102.4 Hz)
        # 3.0 RPM * 2048 steps/rev / 60s = 102.4 Hz. Sampled at 20 Hz, aliases to |102.4 - 5*20| = 2.4 Hz.
        f_noise_30 = 2.4
        buffer_30 = [32000.0 + 1000.0 * math.sin(2.0 * math.pi * f_noise_30 * (n / FS)) for n in range(N)]
        
        mean_30 = sum(buffer_30) / N
        ac_30 = [x - mean_30 for x in buffer_30]
        real_30 = sum(x * c for x, c in zip(ac_30, cos_table))
        imag_30 = sum(x * s for x, s in zip(ac_30, sin_table))
        mag_30 = math.sqrt(real_30*real_30 + imag_30*imag_30) / (N / 2)
        
        # Assertions
        # 3.5 RPM noise leaks heavily into 0.5 Hz, yielding magnitude ~961.6, which is above the threshold
        self.assertGreater(mag_35, code.SIGNAL_THRESHOLD, "3.5 RPM noise must exceed the signal threshold")
        
        # 3.0 RPM noise is safely rejected (magnitude ~71.0), which is below code.SIGNAL_THRESHOLD
        self.assertLess(mag_30, code.SIGNAL_THRESHOLD, "3.0 RPM noise must be below the signal threshold")
        self.assertLess(mag_30, 100.0, "3.0 RPM noise leakage must be less than 100.0")

    def test_dft_transient_rejection(self):
        """Verify that baseline shift transients are detected and rejected, while steady-state beacons pass."""
        FS = code.FS
        N = code.N
        
        # 1. Simulate a step transient (covering the bot: dropping from 4300 to 670 over 10 samples)
        step_signal = [4300.0] * 40
        for i in range(10):
            step_signal.append(4300.0 - (4300.0 - 670.0) * (i / 10))
        step_signal += [670.0] * 60
        
        # Track transient flagging
        window = [32000.0] * N
        mean_history = [32000.0] * 20
        transients = []
        
        for sample in step_signal:
            window.pop(0)
            window.append(sample)
            mean = sum(window) / N
            mean_history.pop(0)
            mean_history.append(mean)
            
            # Replicate code.py transient check
            is_transient = abs(mean - mean_history[0]) > 100.0
            transients.append(is_transient)
            
        # Verify that the transient was detected at some point during the drop
        self.assertTrue(any(transients), "Transient filter must flag a large baseline shift")
        
        # 2. Verify steady-state beacon does NOT trigger transient flagging once buffer is filled
        beacon_signal = [5500.0 + 2500.0 * math.sin(2.0 * math.pi * 0.5 * (t / FS)) for t in range(80)]
        window_b = [32000.0] * N
        mean_history_b = [32000.0] * 20
        transients_b = []
        
        for sample in beacon_signal:
            window_b.pop(0)
            window_b.append(sample)
            mean = sum(window_b) / N
            mean_history_b.pop(0)
            mean_history_b.append(mean)
            is_transient = abs(mean - mean_history_b[0]) > 100.0
            transients_b.append(is_transient)
            
        # After sample 60 (steady state), no transient should be flagged
        self.assertFalse(any(transients_b[60:]), "Steady-state beacon must not trigger the transient filter")

if __name__ == '__main__':
    unittest.main()
