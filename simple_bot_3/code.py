import board
import digitalio
import time
import asyncio
import analogio
import math

try:
    import supervisor
    usb_connected = supervisor.runtime.usb_connected
except (ImportError, AttributeError):
    usb_connected = True

try:
    import neopixel
except ImportError:
    neopixel = None

from motor import StepperMotor

# Logging helper
def log_message(msg):
    print(f"[Follower] {msg}")
    try:
        with open("/log.txt", "a") as f:
            f.write(f"{time.monotonic():.2f}: {msg}\n")
    except OSError:
        pass

# --- Configuration ---
BOT_ROLE = 2            # 1 = Leader (blinks 0.5Hz, no movement), 2 or 3 = Follower
TARGET_FREQ = 0.5       # Blinking frequency of the leader's LEDs (0.5 Hz)
MOVE_DIRECTION = -1     # -1 if robot moves forward on this chassis wiring, 1 if backward
SENSOR_BALANCE = 1.0    # Balance multiplier: > 1.0 boosts Left sensor, < 1.0 boosts Right sensor


# Motor speed limits
MIN_RPM = 3.0
MAX_RPM = 8.0

# DSP / Filter configuration
FS = 20.0               # Sample rate in Hz (every 50ms)
WINDOW_SEC = 2.0        # 2-second window to cover a full 0.5 Hz cycle
N = int(FS * WINDOW_SEC)  # 40 samples

# Detection Thresholds
SIGNAL_THRESHOLD = 600.0  # Min magnitude sum to detect the leader
CLOSE_SIGNAL = 2200.0     # Signal magnitude sum at which we are "Docked" (too close)

# Setup motors
motor1 = StepperMotor(board.GP15, board.GP16, board.GP17, board.GP18, rpm=6.0)
motor2 = StepperMotor(board.GP19, board.GP20, board.GP21, board.GP22, rpm=6.0)

# Setup light sensors (GP26 as Left, GP27 as Right)
light_sensor_left = analogio.AnalogIn(board.GP26)
light_sensor_right = analogio.AnalogIn(board.GP27)

# Setup Sonar Template (Trig=GP11, Echo=GP12) - currently not plugged in
sonar = None
try:
    import adafruit_hcsr04
    sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP11, echo_pin=board.GP12)
    log_message("Sonar sensor template initialized (not plugged in).")
except Exception as e:
    log_message(f"Sonar template initialization skipped/failed: {e}")

# Global variables for DFT magnitudes and follower state
mag_left = 0.0
mag_right = 0.0
state = "SEARCHING"  # States: "SEARCHING", "FOLLOWING", "DOCKED"

# Precompute Cosine and Sine DFT coefficients for efficiency
cos_table = [math.cos(2.0 * math.pi * TARGET_FREQ * (n / FS)) for n in range(N)]
sin_table = [math.sin(2.0 * math.pi * TARGET_FREQ * (n / FS)) for n in range(N)]

# Precompute 1.0 Hz coefficients for NeoPixel crosstalk/reflection rejection
cos_table_10 = [math.cos(2.0 * math.pi * 1.0 * (n / FS)) for n in range(N)]
sin_table_10 = [math.sin(2.0 * math.pi * 1.0 * (n / FS)) for n in range(N)]

# NeoPixel LED Control Task (synchronized blinking + state-based speeds)
async def control_leds():
    if neopixel is None:
        log_message("neopixel module not found. LED control disabled.")
        return

    try:
        # 2 NeoPixels on GP10 (left first, right second in chain)
        pixels = neopixel.NeoPixel(board.GP10, 2, brightness=0.3, auto_write=False)
    except Exception as e:
        log_message(f"Error initializing NeoPixels on GP10: {e}")
        return

    def color_wheel(pos):
        pos = int(pos) % 256
        if pos < 85:
            return (255 - pos * 3, pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return (0, 255 - pos * 3, pos * 3)
        else:
            pos -= 170
            return (pos * 3, 0, 255 - pos * 3)

    log_message("NeoPixel control loop started on GP10.")

    while True:
        t = time.monotonic()

        if BOT_ROLE == 1:
            # Leader: blink at 0.5 Hz (1.0s ON, 1.0s OFF)
            is_on = (t % 2.0) < 1.0
            if is_on:
                pixels[0] = (255, 255, 255)
                pixels[1] = (255, 255, 255)
            else:
                pixels[0] = (0, 0, 0)
                pixels[1] = (0, 0, 0)
        else:
            # State-based blinking with static colors:
            # - DOCKED: Solid Green (DC, no oscillation)
            # - FOLLOWING: Fast blink Blue (4 Hz / 0.25s period)
            # - SEARCHING: Slow blink Yellow/Orange (1 Hz / 1.0s period)
            if state == "DOCKED":
                is_on = True
                color = (0, 255, 0)
            elif state == "FOLLOWING":
                is_on = (t % 0.25) < 0.125
                color = (0, 0, 255)
            else:  # SEARCHING
                is_on = (t % 1.0) < 0.5
                color = (255, 150, 0)

            if is_on:
                pixels[0] = color
                pixels[1] = color
            else:
                pixels[0] = (0, 0, 0)
                pixels[1] = (0, 0, 0)

        try:
            pixels.show()
        except Exception:
            pass

        await asyncio.sleep(0.02)

async def sample_sensors():
    """
    Samples sensors at 20 Hz, AC couples the signal, and computes DFT magnitude
    at both 0.5 Hz (target) and 1.0 Hz (NeoPixel crosstalk blocker).
    Rejects 0.5 Hz signals if the 1.0 Hz component dominates.
    Uses a drift-corrected loop timing pattern to maintain exactly 20.0 Hz.
    """
    global mag_left, mag_right

    # Initialize buffers with reference values
    buffer_left = [32000.0] * N
    buffer_right = [32000.0] * N

    # Initialize mean histories for transient/baseline shift rejection
    mean_history_l = [32000.0] * 20
    mean_history_r = [32000.0] * 20

    # Initialize median histories for spike/glitch rejection
    median_history_l = [32000.0] * 5
    median_history_r = [32000.0] * 5

    sample_count = 0
    next_sample = time.monotonic()

    while True:
        # 1. 5x Burst Oversampling (Averaging) to reduce high-frequency noise
        sum_raw_l = 0.0
        sum_raw_r = 0.0
        for _ in range(5):
            sum_raw_l += float(light_sensor_left.value)
            sum_raw_r += float(light_sensor_right.value)
        raw_l = sum_raw_l / 5.0
        raw_r = sum_raw_r / 5.0

        # 2. Moving Median Filter (length 5) to reject spurious spikes/glitches
        median_history_l.pop(0)
        median_history_l.append(raw_l)
        median_history_r.pop(0)
        median_history_r.append(raw_r)
        
        val_l = sorted(median_history_l)[2]
        val_r = sorted(median_history_r)[2]

        # Shift buffers
        buffer_left.pop(0)
        buffer_left.append(val_l)

        buffer_right.pop(0)
        buffer_right.append(val_r)

        # Efficient single-pass calculation of means
        sum_l = 0.0
        sum_r = 0.0
        for i in range(N):
            sum_l += buffer_left[i]
            sum_r += buffer_right[i]
        mean_l = sum_l / N
        mean_r = sum_r / N

        # Update mean histories
        mean_history_l.pop(0)
        mean_history_l.append(mean_l)
        mean_history_r.pop(0)
        mean_history_r.append(mean_r)

        # Detect baseline shifts (transients) over 1.0 second (20 samples)
        transient_l = abs(mean_l - mean_history_l[0]) > 100.0
        transient_r = abs(mean_r - mean_history_r[0]) > 100.0

        # Single-pass calculation of DFT coefficients for 0.5 Hz and 1.0 Hz
        real_l_05 = 0.0
        imag_l_05 = 0.0
        real_r_05 = 0.0
        imag_r_05 = 0.0

        real_l_10 = 0.0
        imag_l_10 = 0.0
        real_r_10 = 0.0
        imag_r_10 = 0.0

        for i in range(N):
            ac_val_l = buffer_left[i] - mean_l
            ac_val_r = buffer_right[i] - mean_r

            # 0.5 Hz coefficients
            c05 = cos_table[i]
            s05 = sin_table[i]
            real_l_05 += ac_val_l * c05
            imag_l_05 += ac_val_l * s05
            real_r_05 += ac_val_r * c05
            imag_r_05 += ac_val_r * s05

            # 1.0 Hz coefficients
            c10 = cos_table_10[i]
            s10 = sin_table_10[i]
            real_l_10 += ac_val_l * c10
            imag_l_10 += ac_val_l * s10
            real_r_10 += ac_val_r * c10
            imag_r_10 += ac_val_r * s10

        mag_l_05 = math.sqrt(real_l_05 * real_l_05 + imag_l_05 * imag_l_05) / (N / 2)
        mag_r_05 = math.sqrt(real_r_05 * real_r_05 + imag_r_05 * imag_r_05) / (N / 2)

        mag_l_10 = math.sqrt(real_l_10 * real_l_10 + imag_l_10 * imag_l_10) / (N / 2)
        mag_r_10 = math.sqrt(real_r_10 * real_r_10 + imag_r_10 * imag_r_10) / (N / 2)

        # Crosstalk blocking: if the 1.0 Hz signal is stronger than 0.5 Hz (or a significant fraction),
        # it is our own status light or crosstalk, so reject it.
        # Also reject signal if we are undergoing a baseline transient (ambient light shift).
        if mag_l_10 > 0.8 * mag_l_05 or transient_l:
            mag_left = 0.0
        else:
            mag_left = mag_l_05

        if mag_r_10 > 0.8 * mag_r_05 or transient_r:
            mag_right = 0.0
        else:
            mag_right = mag_r_05

        # Increment sample counter and log spectrum periodically
        sample_count += 1
        if sample_count % 10 == 0:
            ac_l = [x - mean_l for x in buffer_left]
            ac_r = [x - mean_r for x in buffer_right]
            mags_l = {}
            mags_r = {}
            for k in range(1, 7):
                real_l = 0.0
                imag_l = 0.0
                real_r = 0.0
                imag_r = 0.0
                for i in range(N):
                    angle = 2.0 * math.pi * k * i / N
                    cos_val = math.cos(angle)
                    sin_val = math.sin(angle)
                    real_l += ac_l[i] * cos_val
                    imag_l += ac_l[i] * sin_val
                    real_r += ac_r[i] * cos_val
                    imag_r += ac_r[i] * sin_val
                mags_l[k] = math.sqrt(real_l * real_l + imag_l * imag_l) / (N / 2)
                mags_r[k] = math.sqrt(real_r * real_r + imag_r * imag_r) / (N / 2)
            print(f"[Freq Log] Raw L: {val_l:.0f}, R: {val_r:.0f} | Mean L: {mean_l:.1f}, R: {mean_r:.1f}")
            print(f"[Freq Log] L spectrum -> 0.5Hz: {mags_l[1]:.1f}, 1.0Hz: {mags_l[2]:.1f}, 1.5Hz: {mags_l[3]:.1f}, 2.0Hz: {mags_l[4]:.1f}, 2.5Hz: {mags_l[5]:.1f}, 3.0Hz: {mags_l[6]:.1f}")
            print(f"[Freq Log] R spectrum -> 0.5Hz: {mags_r[1]:.1f}, 1.0Hz: {mags_r[2]:.1f}, 1.5Hz: {mags_r[3]:.1f}, 2.0Hz: {mags_r[4]:.1f}, 2.5Hz: {mags_r[5]:.1f}, 3.0Hz: {mags_r[6]:.1f}")

        # Precise drift-corrected timing loop
        next_sample += 1.0 / FS
        now = time.monotonic()
        sleep_dur = next_sample - now
        if sleep_dur > 0:
            await asyncio.sleep(sleep_dur)
        else:
            # Under heavy load/jitter, reset next_sample target to now
            next_sample = now
            await asyncio.sleep(0)

async def steer_robot():
    """
    State machine that controls motor speed and direction based on DFT magnitudes.
    Uses Braitenberg-like emergent properties: speeds up the fainter side to turn.
    Includes a signal-loss grace period to prevent stopping/spinning immediately after turns.
    Calibrates sensor balance once at boot for 2.0s and locks it forever.
    """
    global state
    last_known_dir = 1  # 1 = Left, -1 = Right
    lost_signal_counter = 0
    
    # Store last calculated RPMs for use during the blind-follow grace period
    last_rpm1 = 5.0
    last_rpm2 = 5.0
    searching_confirm_counter = 0
    
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

    log_message("Starting 2-second boot calibration sequence (staying stationary)...")
    boot_left = []
    boot_right = []
    # Collect samples at 20 Hz (every 50ms) for 2.0 seconds (40 samples)
    for _ in range(40):
        boot_left.append(float(light_sensor_left.value))
        boot_right.append(float(light_sensor_right.value))
        await asyncio.sleep(0.05)
        
    avg_left = sum(boot_left) / 40.0
    avg_right = sum(boot_right) / 40.0
    
    # Calculate fixed balance factor to balance sensor gains
    if avg_left > 1.0:
        auto_balance = avg_right / avg_left
    else:
        auto_balance = 1.0
        
    # Constrain balance factor between 0.5 and 2.0 for safety
    auto_balance = max(0.5, min(2.0, auto_balance))
    log_message(f"Boot calibration complete: Avg L={avg_left:.1f}, Avg R={avg_right:.1f} | Auto-Balance locked at: {auto_balance:.3f}")

    log_message("Warming up DSP filters (3 seconds)...")
    await asyncio.sleep(3.0)
    log_message("Ready to follow 0.5 Hz beacon!")
    
    while True:
        # Apply both manual baseline and dynamic auto-balance
        balanced_left = mag_left * SENSOR_BALANCE * auto_balance
        balanced_right = mag_right
        total_signal = balanced_left + balanced_right
        
        # Track last known direction of the target if we have a decent signal
        if total_signal >= SIGNAL_THRESHOLD:
            last_known_dir = 1 if balanced_left > balanced_right else -1
            if state == "SEARCHING":
                searching_confirm_counter += 1
        else:
            searching_confirm_counter = 0

        # Check proximity threshold (Docking)
        if total_signal >= CLOSE_SIGNAL:
            state = "DOCKED"
            lost_signal_counter = 0
            set_motors(False, False, 0, 0)
            print(f"[DOCKED] Target reached. Signal: {total_signal:.0f} (Left: {balanced_left:.1f}, Right: {balanced_right:.1f})")
            
        elif total_signal < SIGNAL_THRESHOLD:
            # Check if we should use the blind-follow grace period before turning/spinning
            if state == "FOLLOWING" and lost_signal_counter < 8:  # 8 iterations * 100ms = 800ms grace period
                lost_signal_counter += 1
                # Continue driving at last known speed
                motor1.set_rpm(last_rpm1)
                motor2.set_rpm(last_rpm2)
                set_motors(True, True, MOVE_DIRECTION, MOVE_DIRECTION)
                print(f"[GRACE PERIOD] Blind following... retry {lost_signal_counter}/8. Signal: {total_signal:.1f}")
            else:
                # Target lost: Spin slowly in place to search
                state = "SEARCHING"
                motor1.set_rpm(3.0)
                motor2.set_rpm(3.0)
                # Spin in last known direction of target (correcting search spin direction)
                set_motors(True, True, -MOVE_DIRECTION * last_known_dir, MOVE_DIRECTION * last_known_dir)
                print(f"[SEARCHING] Target lost. Signal: {total_signal:.1f}. Spinning {'LEFT' if last_known_dir > 0 else 'RIGHT'} to find leader...")
            
        else:
            # Target in view: Follow leader using emergent steering behaviors
            if state == "SEARCHING" and searching_confirm_counter < 5:
                # Target detected but not confirmed yet: spin slowly and stay in SEARCHING
                state = "SEARCHING"
                motor1.set_rpm(3.0)
                motor2.set_rpm(3.0)
                set_motors(True, True, -MOVE_DIRECTION * last_known_dir, MOVE_DIRECTION * last_known_dir)
                print(f"[SEARCHING] Target detected, confirming... ({searching_confirm_counter}/5) Signal: {total_signal:.1f}")
            else:
                state = "FOLLOWING"
                lost_signal_counter = 0
            
            # Proportional difference normalized by total signal
            diff = balanced_left - balanced_right
            norm_diff = diff / total_signal if total_signal > 0 else 0.0
            
            # Since both motors run forward (even if MOVE_DIRECTION is negative),
            # speeding up the right motor turns the robot left, and speeding up the left motor turns it right.
            # No sign inversion is needed for steering.
            effective_diff = norm_diff
            
            steer_gain = 4.5
            base_rpm = 5.0
            
            # Emergent behavior: speed up the motor on the fainter side to turn towards the brighter side.
            # Very small dead-band (0.005) to correct even tiny parallel drift offsets.
            if effective_diff > 0.005:
                # Right is fainter (Left is brighter): speed up Right motor (motor2) to turn Left
                rpm1 = base_rpm
                rpm2 = base_rpm + (effective_diff * steer_gain)
            elif effective_diff < -0.005:
                # Left is fainter (Right is brighter): speed up Left motor (motor1) to turn Right
                rpm1 = base_rpm + (abs(effective_diff) * steer_gain)
                rpm2 = base_rpm
            else:
                # Balanced: drive straight at base speed
                rpm1 = base_rpm
                rpm2 = base_rpm
                
            # Apply safety bounds
            rpm1 = max(MIN_RPM, min(MAX_RPM, rpm1))
            rpm2 = max(MIN_RPM, min(MAX_RPM, rpm2))
            
            # Cache last known speed for the grace period
            last_rpm1 = rpm1
            last_rpm2 = rpm2
            
            motor1.set_rpm(rpm1)
            motor2.set_rpm(rpm2)
            
            set_motors(True, True, MOVE_DIRECTION, MOVE_DIRECTION)
            print(f"[FOLLOWING] Mag: L={mag_left:4.0f} R={mag_right:4.0f} | Auto-Balance: {auto_balance:.3f} | Locked: True | Diff: {norm_diff:+.2f} | RPM: L={rpm1:.1f} R={rpm2:.1f}")

        await asyncio.sleep(0.1)

async def main():
    if BOT_ROLE == 1:
        log_message("Starting leader main loop (Role 1)...")
        await control_leds()
    else:
        log_message(f"Starting follower main loop (Role {BOT_ROLE})...")
        asyncio.create_task(motor1.run())
        asyncio.create_task(motor2.run())
        asyncio.create_task(control_leds())
        asyncio.create_task(sample_sensors())
        await steer_robot()

if __name__ == "__main__":
    asyncio.run(main())
