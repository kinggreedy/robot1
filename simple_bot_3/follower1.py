import board
import digitalio
import time
import asyncio
import analogio
import math

from motor import StepperMotor

# --- Configuration ---
TARGET_FREQ = 0.5       # Blinking frequency of the leader's LEDs (0.5 Hz)
MOVE_DIRECTION = 1     # -1 if robot moves forward on this chassis wiring, 1 if backward

# Motor speed limits
MIN_RPM = 3.0
MAX_RPM = 8.0

# DSP / Filter configuration
FS = 20.0               # Sample rate in Hz (every 50ms)
WINDOW_SEC = 2.0        # 2-second window to cover a full 0.5 Hz cycle
N = int(FS * WINDOW_SEC)  # 40 samples

# Detection Thresholds
SIGNAL_THRESHOLD = 200.0  # Min magnitude sum to detect the leader
CLOSE_SIGNAL = 2200.0     # Signal magnitude sum at which we are "Docked" (too close)

# Setup motors
motor1 = StepperMotor(board.GP15, board.GP16, board.GP17, board.GP18, rpm=6.0)
motor2 = StepperMotor(board.GP19, board.GP20, board.GP21, board.GP22, rpm=6.0)

# Setup light sensors (GP26 as Left, GP27 as Right)
light_sensor_left = analogio.AnalogIn(board.GP26)
light_sensor_right = analogio.AnalogIn(board.GP27)

# Setup LED (GP10 as digital status output)
led = digitalio.DigitalInOut(board.GP10)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# Global variables for DFT magnitudes and follower state
mag_left = 0.0
mag_right = 0.0
state = "SEARCHING"  # States: "SEARCHING", "FOLLOWING", "DOCKED"

# Precompute Cosine and Sine DFT coefficients for efficiency
cos_table = [math.cos(2.0 * math.pi * TARGET_FREQ * (n / FS)) for n in range(N)]
sin_table = [math.sin(2.0 * math.pi * TARGET_FREQ * (n / FS)) for n in range(N)]

def log_message(msg):
    print(f"[Follower] {msg}")

async def sample_sensors():
    """
    Samples sensors at 20 Hz, AC couples the signal to remove ambient room light,
    and computes the DFT magnitude at exactly 0.5 Hz.
    """
    global mag_left, mag_right
    
    # Initialize buffers with reference values
    buffer_left = [32000.0] * N
    buffer_right = [32000.0] * N
    
    while True:
        # Read raw sensor values (0-65535)
        val_l = float(light_sensor_left.value)
        val_r = float(light_sensor_right.value)
        
        # Shift buffers
        buffer_left.pop(0)
        buffer_left.append(val_l)
        
        buffer_right.pop(0)
        buffer_right.append(val_r)
        
        # Calculate running means (ambient level)
        mean_l = sum(buffer_left) / N
        mean_r = sum(buffer_right) / N
        
        # Subtract mean to isolate AC signal (removes constant light)
        ac_l = [x - mean_l for x in buffer_left]
        ac_r = [x - mean_r for x in buffer_right]
        
        # Compute Single-Point DFT at TARGET_FREQ (0.5 Hz)
        real_l = sum(x * c for x, c in zip(ac_l, cos_table))
        imag_l = sum(x * s for x, s in zip(ac_l, sin_table))
        mag_left = math.sqrt(real_l**2 + imag_l**2) / (N / 2)
        
        real_r = sum(x * c for x, c in zip(ac_r, cos_table))
        imag_r = sum(x * s for x, s in zip(ac_r, sin_table))
        mag_right = math.sqrt(real_r**2 + imag_r**2) / (N / 2)
        
        await asyncio.sleep(1.0 / FS)

async def control_indicator():
    """
    Updates the on-board GP10 LED status.
    - Searching: Blinks slowly (1 Hz)
    - Following: Blinks fast (4 Hz)
    - Docked: Solid ON
    """
    while True:
        t = time.monotonic()
        if state == "DOCKED":
            led.value = True
        elif state == "FOLLOWING":
            # Fast blink
            led.value = (t % 0.25) < 0.125
        else:
            # Slow blink
            led.value = (t % 1.0) < 0.5
        await asyncio.sleep(0.05)

async def steer_robot():
    """
    State machine that controls motor speed and direction based on DFT magnitudes.
    """
    global state
    last_known_dir = 1  # 1 = Left, -1 = Right
    
    # Track motor state to avoid redundant command calls
    m1_active = False
    m2_active = False
    
    def set_motors(run_m1, run_m2, dir_m1, dir_m2):
        nonlocal m1_active, m2_active
        if run_m1:
            motor1.move_forever(dir_m1)
            m1_active = True
        elif m1_active:
            motor1.stop()
            m1_active = False
            
        if run_m2:
            motor2.move_forever(dir_m2)
            m2_active = True
        elif m2_active:
            motor2.stop()
            m2_active = False

    log_message("Warming up DSP filters (3 seconds)...")
    await asyncio.sleep(3.0)
    log_message("Ready to follow 0.5 Hz beacon!")
    
    while True:
        total_signal = mag_left + mag_right
        
        # Track last known direction of the target if we have a decent signal
        if total_signal >= SIGNAL_THRESHOLD:
            last_known_dir = 1 if mag_left > mag_right else -1

        # Check proximity threshold (Docking)
        if total_signal >= CLOSE_SIGNAL:
            state = "DOCKED"
            set_motors(False, False, 0, 0)
            print(f"[DOCKED] Target reached. Signal: {total_signal:.0f} (Left: {mag_left:.1f}, Right: {mag_right:.1f})")
            
        elif total_signal < SIGNAL_THRESHOLD:
            # Target lost: Spin slowly in place to search
            state = "SEARCHING"
            motor1.set_rpm(3.5)
            motor2.set_rpm(3.5)
            # Spin in last known direction of target
            set_motors(True, True, MOVE_DIRECTION * last_known_dir, -MOVE_DIRECTION * last_known_dir)
            print(f"[SEARCHING] Target lost. Signal: {total_signal:.1f}. Spinning {'LEFT' if last_known_dir > 0 else 'RIGHT'} to find leader...")
            
        else:
            # Target in view: Follow leader
            state = "FOLLOWING"
            
            # Proportional Braitenberg steering error normalized by total signal
            error = (mag_left - mag_right) / total_signal
            steer_gain = 4.5
            base_rpm = 5.5
            
            # Adjust motor speeds proportionally based on steering error
            if error > 0.08:
                rpm1 = base_rpm - (error * steer_gain)
                rpm2 = base_rpm
            elif error < -0.08:
                rpm1 = base_rpm
                rpm2 = base_rpm - (abs(error) * steer_gain)
            else:
                rpm1 = base_rpm
                rpm2 = base_rpm
                
            # Apply safety bounds
            rpm1 = max(MIN_RPM, min(MAX_RPM, rpm1))
            rpm2 = max(MIN_RPM, min(MAX_RPM, rpm2))
            
            motor1.set_rpm(rpm1)
            motor2.set_rpm(rpm2)
            
            set_motors(True, True, MOVE_DIRECTION, MOVE_DIRECTION)
            print(f"[FOLLOWING] Mag: L={mag_left:4.0f} R={mag_right:4.0f} | Error: {error:+.2f} | RPM: L={rpm1:.1f} R={rpm2:.1f}")
            
        await asyncio.sleep(0.1)

async def main():
    log_message("Starting follower main loop...")
    asyncio.create_task(motor1.run())
    asyncio.create_task(motor2.run())
    asyncio.create_task(control_indicator())
    asyncio.create_task(sample_sensors())
    await steer_robot()

if __name__ == "__main__":
    asyncio.run(main())

