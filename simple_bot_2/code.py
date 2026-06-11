import board
import digitalio
import time
import asyncio
import analogio

from motor import StepperMotor

# RPM Configuration
MIN_RPM = 5.0   # Minimum RPM when slowing down near light source
MAX_RPM = 8.0   # Maximum RPM (user requested 8 max RPM)

# Calibration parameters
CALIBRATION_STEPS = 7022  # Number of steps to complete a full 360-degree rotation
THRESHOLD_FRACTION = 0.6  # Threshold fraction above ambient
UNIFORM_LIGHT_THRESHOLD = 3000  # Minimum difference to distinguish a light source from ambient
LIGHT_POLARITY = 1        # 1 if more light increases sensor value (e.g. photodiode), -1 if it decreases
FLASHLIGHT_MARGIN = 4000  # Margin above max ambient light to trigger flashlight mode

# Motor direction for forward movement
MOVE_DIRECTION = -1  # Set to -1 if robot moves backward during following, 1 if forward


#Setup motors
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

# Setup light sensors (GP26 and GP27 as analog inputs)
light_sensor_left = analogio.AnalogIn(board.GP26)
light_sensor_right = analogio.AnalogIn(board.GP27)

# Setup LED (GP10 as digital output)
led = digitalio.DigitalInOut(board.GP10)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# Setup Sonar (Trig=GP13, Echo=GP12)
sonar = None
try:
    import adafruit_hcsr04
    sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP13, echo_pin=board.GP12)
    print("Sonar sensor initialized successfully.")
except Exception as e:
    print(f"Error initializing Sonar sensor: {e}")

# Setup Color Sensor APDS-9960 (I2C: SCL=GP1, SDA=GP0)
apds = None
try:
    import busio
    from adafruit_apds9960.apds9960 import APDS9960
    i2c = busio.I2C(board.GP1, board.GP0)
    apds = APDS9960(i2c)
    apds.enable_color = True
    print("APDS-9960 Color sensor initialized successfully.")
except Exception as e:
    print(f"Error initializing APDS-9960 Color sensor: {e}")

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

async def calibrate_sensors():
    global min_left, max_left, min_right, max_right
    global threshold_left, threshold_right, calibrated, light_source_present

    print("Starting calibration... Spinning 360 degrees to scan ambient light.")

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
        print(f"Brightest source detected! Rotating back to direction {best_pos}.")
        # Rotate back to face the brightest source
        motor1.move_to(best_pos)
        motor2.move_to(-best_pos)
        await wait_for_motors(motor1, motor2)
    else:
        light_source_present = False
        print("Environment is uniform. Sticking to stationary mode until a bright light is detected.")

    calibrated = True
    print("Calibration finished!")
    print(f"Left sensor: min={min_left}, max={max_left}, threshold={threshold_left:.1f}")
    print(f"Right sensor: min={min_right}, max={max_right}, threshold={threshold_right:.1f}")

async def wait_for_motors(*motors):
    #Wait for the motors to finish rotating
    while any(m.busy for m in motors):
        await asyncio.sleep(0)


#Example code for running a sequence of movements
async def demo():
    # Wait until calibration is complete
    while not calibrated:
        await asyncio.sleep(0.1)

    # The read_sensors loop handles motor movement based on sensor inputs
    while True:
        await asyncio.sleep(1)

def update_motor_behaviors(left_val, right_val):
    # Normalize values relative to calibrated ambient range to handle mounting differences
    range_l = max(1, max_left - min_left)
    range_r = max(1, max_right - min_right)
    norm_l = (left_val - min_left) / range_l
    norm_r = (right_val - min_right) / range_r

    # Base speeds
    rpm1 = MAX_RPM
    rpm2 = MAX_RPM

    # Steer towards the side with the higher normalized reading
    # Slow down the motor on the brighter side (inverse steering)
    diff = norm_l - norm_r
    if diff > 0:
        # Turn left: slow down left motor
        rpm1 = MAX_RPM - diff * 6.0  # Steer gain
    else:
        # Turn right: slow down right motor
        rpm2 = MAX_RPM - abs(diff) * 6.0

    # Slow down if both sensors are close to a bright source (large norm values)
    avg_norm = (norm_l + norm_r) / 2.0
    if avg_norm > 1.2:
        scale = max(0.2, 1.0 - (avg_norm - 1.2) * 0.5)
        rpm1 *= scale
        rpm2 *= scale

    # Apply safety bounds
    rpm1 = max(MIN_RPM, min(MAX_RPM, rpm1))
    rpm2 = max(MIN_RPM, min(MAX_RPM, rpm2))

    # Set motor speeds and start them if not running
    motor1.set_rpm(rpm1)
    if not motor1.run_forever:
        motor1.move_forever(MOVE_DIRECTION)

    motor2.set_rpm(rpm2)
    if not motor2.run_forever:
        motor2.move_forever(MOVE_DIRECTION)

    return rpm1, rpm2

#Loop for reading sensor values and logging/printing
async def read_sensors():
    while True:
        # Read raw light values (0-65535)
        left_val = light_sensor_left.value
        right_val = light_sensor_right.value

        # Read sonar distance
        sonar_dist = "N/A"
        if sonar is not None:
            try:
                sonar_dist = f"{sonar.distance:.1f} cm"
            except Exception as e:
                sonar_dist = f"Error: {e}"

        # Read color data
        color_rgbc = "N/A"
        if apds is not None:
            try:
                r, g, b, c = apds.color_data
                color_rgbc = f"R={r}, G={g}, B={b}, C={c}"
            except Exception as e:
                color_rgbc = f"Error: {e}"

        # Print all values to log/console
        print(f"Time: {time.monotonic():.2f}s | Light: L={left_val}, R={right_val} | Sonar: {sonar_dist} | Color: {color_rgbc}")

        await asyncio.sleep(0.5)


async def main():
    # Start reading sensors directly without calibration or motor movement
    asyncio.create_task(read_sensors())
    # Keep the program running
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
