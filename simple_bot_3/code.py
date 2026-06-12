import board
import digitalio
import time
import asyncio
import analogio

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
    print(f"[LOG] {msg}")
    try:
        with open("/log.txt", "a") as f:
            f.write(f"{time.monotonic():.2f}: {msg}\n")
    except OSError:
        pass


# RPM Configuration
MIN_RPM = 5.0   # Minimum RPM when slowing down near light source
MAX_RPM = 8.0   # Maximum RPM

# Calibration parameters
CALIBRATION_STEPS = 8192  # Number of steps to complete a full 360-degree rotation (2048 per 90 deg)
THRESHOLD_FRACTION = 0.6  # Threshold fraction above ambient
UNIFORM_LIGHT_THRESHOLD = 3000  # Minimum difference to distinguish a light source from ambient
LIGHT_POLARITY = 1        # 1 if more light increases sensor value (e.g. photodiode), -1 if it decreases
FLASHLIGHT_MARGIN = 4000  # Margin above max ambient light to trigger flashlight mode

# Motor direction for forward movement
MOVE_DIRECTION = -1  # Set to -1 if robot moves backward during following, 1 if forward


# Setup motors
motor1 = StepperMotor(
    board.GP15,
    board.GP16,
    board.GP17,
    board.GP18,
    rpm=12
)

motor2 = StepperMotor(
    board.GP19,
    board.GP20,
    board.GP21,
    board.GP22,
    rpm=12
)

# Setup light sensors (GP13 as Left, GP14 as Right)
class SmartLightSensor:
    def __init__(self, pin):
        self._analog = None
        self._digital = None
        pin_name = str(pin).upper()
        # On RP2040 chip, only GP26, GP27, GP28, GP29 are ADC analog pins.
        if any(adc in pin_name for adc in ["GP26", "GP27", "GP28", "GP29"]):
            try:
                self._analog = analogio.AnalogIn(pin)
                log_message(f"Initialized analog light sensor on {pin_name}")
                return
            except Exception:
                pass
        
        self._digital = digitalio.DigitalInOut(pin)
        self._digital.direction = digitalio.Direction.INPUT
        log_message(f"Initialized digital light sensor on {pin_name}")

    @property
    def value(self):
        if self._analog is not None:
            return self._analog.value
        else:
            # Active-low sensor: returns 65535 when light is detected (DO goes LOW / False), 0 otherwise
            return 0 if self._digital.value else 65535

light_sensor_left = SmartLightSensor(board.GP13)
light_sensor_right = SmartLightSensor(board.GP14)


# Setup Sonar Template (Trig=GP11, Echo=GP12) - currently not plugged in
sonar = None
try:
    import adafruit_hcsr04
    sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP11, echo_pin=board.GP12)
    log_message("Sonar sensor template initialized (not plugged in).")
except Exception as e:
    log_message(f"Sonar template initialization skipped/failed: {e}")

# Calibration state
min_left, max_left = 65535, 0
min_right, max_right = 65535, 0
threshold_left = 0.0
threshold_right = 0.0
calibrated = False
light_source_present = False

# Guided mode state (triggered when flashlight is detected)
guided_mode = False

# End indication state
INDICATE_END = False
stop_time = None


# NeoPixel LED Control Task
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
        
        if INDICATE_END:
            # Strobe red alert when finished / stopped
            is_on = (t % 0.4) < 0.2
            if is_on:
                pixels[0] = (255, 0, 0)
                pixels[1] = (255, 0, 0)
            else:
                pixels[0] = (0, 0, 0)
                pixels[1] = (0, 0, 0)
        else:
            # Normal: full beautiful RGB colorwheel cycle, blinking 1s on / 1s off in perfect sync (2s cycle)
            both_on = (t % 2.0) < 1.0
            hue = int(t * 50) % 256
            
            if both_on:
                pixels[0] = color_wheel(hue)
                pixels[1] = color_wheel((hue + 128) % 256)
            else:
                pixels[0] = (0, 0, 0)
                pixels[1] = (0, 0, 0)
                
        try:
            pixels.show()
        except Exception:
            pass
            
        await asyncio.sleep(0.02)


async def calibrate_sensors():
    global min_left, max_left, min_right, max_right
    global threshold_left, threshold_right, calibrated, light_source_present

    log_message("Starting calibration... Spinning 360 degrees to scan ambient light.")

    # Spin in place: Motor 1 forward, Motor 2 reverse
    motor1.set_rpm(6.0)
    motor2.set_rpm(6.0)
    motor1.move(CALIBRATION_STEPS)
    motor2.move(-CALIBRATION_STEPS)

    min_l, max_l = 65535, 0
    min_r, max_r = 65535, 0

    readings_history = []
    best_pos = 0
    best_smoothed_val = 0
    min_smoothed_val = 65535

    while motor1.busy or motor2.busy:
        left_val = light_sensor_left.value
        right_val = light_sensor_right.value

        # Track raw min/max for normalization
        if left_val < min_l: min_l = left_val
        if left_val > max_l: max_l = left_val
        if right_val < min_r: min_r = right_val
        if right_val > max_r: max_r = right_val

        # Calculate current average light and keep a history of size 5 for smoothing
        avg_val = (left_val + right_val) / 2.0
        readings_history.append(avg_val)
        if len(readings_history) > 5:
            readings_history.pop(0)

        # Smooth reading to prevent peak light error
        smoothed_val = sum(readings_history) / len(readings_history)

        current_pos = motor1.position
        if smoothed_val > best_smoothed_val:
            best_smoothed_val = smoothed_val
            best_pos = current_pos

        if len(readings_history) == 5 and smoothed_val < min_smoothed_val:
            min_smoothed_val = smoothed_val

        print(f"Scanning... Pos: {current_pos}, L: {left_val}, R: {right_val}, Smoothed: {smoothed_val:.1f}")
        await asyncio.sleep(0.05)

    min_left, max_left = min_l, max_l
    min_right, max_right = min_r, max_r

    # Calculate thresholds
    range_l = max_left - min_left
    range_r = max_right - min_right
    threshold_left = min_left + range_l * THRESHOLD_FRACTION
    threshold_right = min_right + range_r * THRESHOLD_FRACTION

    # Check if the environment has a distinct light source during scan
    if (best_smoothed_val - min_smoothed_val) >= UNIFORM_LIGHT_THRESHOLD:
        light_source_present = True
        log_message(f"Brightest source detected! Rotating back to direction {best_pos}.")
        # Rotate back to face the brightest source
        motor1.move_to(best_pos)
        motor2.move_to(-best_pos)
        await wait_for_motors(motor1, motor2)
    else:
        light_source_present = False
        log_message("Environment is uniform. Sticking to stationary mode until a bright light is detected.")

    calibrated = True
    log_message("Calibration finished!")
    log_message(f"Left sensor: min={min_left}, max={max_left}, threshold={threshold_left:.1f}")
    log_message(f"Right sensor: min={min_right}, max={max_right}, threshold={threshold_right:.1f}")

async def wait_for_motors(*motors):
    #Wait for the motors to finish rotating
    while any(m.busy for m in motors):
        await asyncio.sleep(0.01)

def update_motor_behaviors(left_val, right_val):
    # Determine if light is detected above/below threshold
    light_detected_l = False
    light_detected_r = False
    rpm1 = 0.0
    rpm2 = 0.0

    if LIGHT_POLARITY == 1:
        if left_val > threshold_left:
            light_detected_l = True
            denom = max(1, max_left - threshold_left)
            rpm1 = MIN_RPM + (MAX_RPM - MIN_RPM) * ((left_val - threshold_left) / denom)

        if right_val > threshold_right:
            light_detected_r = True
            denom = max(1, max_right - threshold_right)
            rpm2 = MIN_RPM + (MAX_RPM - MIN_RPM) * ((right_val - threshold_right) / denom)
    else:
        if left_val < threshold_left:
            light_detected_l = True
            denom = max(1, threshold_left - min_left)
            rpm1 = MIN_RPM + (MAX_RPM - MIN_RPM) * ((threshold_left - left_val) / denom)

        if right_val < threshold_right:
            light_detected_r = True
            denom = max(1, threshold_right - min_right)
            rpm2 = MIN_RPM + (MAX_RPM - MIN_RPM) * ((threshold_right - right_val) / denom)

    # Apply motor controls based on light detection
    if light_detected_l:
        rpm1 = max(MIN_RPM, min(MAX_RPM, rpm1))
        motor1.set_rpm(rpm1)
        if not motor1.run_forever:
            motor1.move_forever(MOVE_DIRECTION)
    else:
        if motor1.run_forever:
            motor1.stop()
        rpm1 = 0.0

    if light_detected_r:
        rpm2 = max(MIN_RPM, min(MAX_RPM, rpm2))
        motor2.set_rpm(rpm2)
        if not motor2.run_forever:
            motor2.move_forever(MOVE_DIRECTION)
    else:
        if motor2.run_forever:
            motor2.stop()
        rpm2 = 0.0

    return rpm1, rpm2

#Loop for reading sensor values and updating motor speed
async def read_sensors():
    global light_source_present, guided_mode, INDICATE_END, stop_time
    while True:
        if not calibrated:
            await asyncio.sleep(0.1)
            continue

        # Read raw light values (0-65535)
        left_val = light_sensor_left.value
        right_val = light_sensor_right.value

        # Detect if the super bright flashlight is active
        flashlight_detected = (left_val > max_left + FLASHLIGHT_MARGIN) or (right_val > max_right + FLASHLIGHT_MARGIN)

        # Once flashlight is detected left or right, switch to guided mode
        if flashlight_detected:
            guided_mode = True

        # Determine if we should move:
        # - In guided mode: only move if the flashlight is currently detected (super bright light active)
        # - Otherwise: move if there was a light source during calibration and we are above threshold
        if guided_mode:
            should_move = flashlight_detected
        else:
            should_move = light_source_present and ((left_val > threshold_left) or (right_val > threshold_right))

        if should_move:
            rpm1, rpm2 = update_motor_behaviors(left_val, right_val)
            # Reset stop timer and end indicator since we are moving
            stop_time = None
            INDICATE_END = False
        else:
            # Stop both motors if no light is detected above thresholds or initial condition had no source
            if motor1.run_forever:
                motor1.stop()
            if motor2.run_forever:
                motor2.stop()
            rpm1 = 0.0
            rpm2 = 0.0

            # Start/check stop timer (tracks consecutive seconds of being stopped)
            if stop_time is None:
                stop_time = time.monotonic()
            elif time.monotonic() - stop_time >= 10.0:
                INDICATE_END = True

        print(f"Sensors: L={left_val} (RPM={rpm1:.2f}), R={right_val} (RPM={rpm2:.2f}) | Guided={guided_mode} | Active={should_move} | End={INDICATE_END}")

        await asyncio.sleep(0.05)


async def rotate_360():
    log_message("Starting 360-degree rotation task...")

    # Set speed for both motors
    motor1.set_rpm(6.0)
    motor2.set_rpm(6.0)

    # Spin in place: Motor 1 forward, Motor 2 reverse
    log_message(f"Commanding motor1 to move {CALIBRATION_STEPS} steps and motor2 to move {-CALIBRATION_STEPS} steps.")
    motor1.move(CALIBRATION_STEPS)
    motor2.move(-CALIBRATION_STEPS)

    # Wait for completion while logging progress
    while motor1.busy or motor2.busy:
        log_message(f"Progress: Motor 1 pos = {motor1.position}/{motor1.target}, Motor 2 pos = {motor2.position}/{motor2.target}")
        await asyncio.sleep(0.5)

    log_message("360-degree rotation task complete. Stopping motors.")
    motor1.stop()
    motor2.stop()


async def main():
    log_message("Booting up simple_bot_1...")

    asyncio.create_task(motor1.run())
    asyncio.create_task(motor2.run())
    asyncio.create_task(control_leds())

    # Rotate 360 degrees only (as configured by USER)
    await rotate_360()

    log_message("Finished 360-degree rotation. Entering idle loop.")
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())


