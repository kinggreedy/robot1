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
CALIBRATION_STEPS = 8192  # Number of steps to complete a full 360-degree rotation (2048 per 90 deg)
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

# Setup LEDs (GP10 as Red LED, GP11 as Yellow LED)
led_red = digitalio.DigitalInOut(board.GP10)
led_red.direction = digitalio.Direction.OUTPUT
led_red.value = False

led_yellow = digitalio.DigitalInOut(board.GP11)
led_yellow.direction = digitalio.Direction.OUTPUT
led_yellow.value = False

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
except Exception:
    pass

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

# Helper to get filtered sonar readings
def get_filtered_sonar(samples=3):
    readings = []
    for _ in range(samples):
        try:
            dist = sonar.distance
            # Filter out jumps (e.g. 800cm if too close) and invalid ranges
            if 2.0 <= dist <= 250.0:
                readings.append(dist)
        except Exception:
            pass
        time.sleep(0.005)
        
    if not readings:
        return None
    
    # Return median
    readings.sort()
    return readings[len(readings) // 2]


# Rate-of-change robust filter for straight driving
last_valid_dist = None
glitch_count = 0

def get_robust_sonar():
    global last_valid_dist, glitch_count
    current = get_filtered_sonar()
    if current is None:
        return None
        
    if last_valid_dist is not None:
        diff = abs(current - last_valid_dist)
        if diff > 15.0:
            glitch_count += 1
            if glitch_count < 3:
                # Discard sudden jump, return last valid value
                return last_valid_dist
            else:
                # Persistent change, accept it
                last_valid_dist = current
                glitch_count = 0
                return current
        else:
            last_valid_dist = current
            glitch_count = 0
            return current
    else:
        last_valid_dist = current
        glitch_count = 0
        return current

# Motor movement helpers
async def drive_steps(steps, direction=MOVE_DIRECTION):
    motor1.set_rpm(MAX_RPM)
    motor2.set_rpm(MAX_RPM)
    motor1.move(steps * direction)
    motor2.move(steps * direction)
    await wait_for_motors(motor1, motor2)

async def turn_degrees(degrees):
    motor1.set_rpm(MAX_RPM)
    motor2.set_rpm(MAX_RPM)
    steps = int(degrees * (CALIBRATION_STEPS / 360.0))
    motor1.move(steps)
    motor2.move(-steps)
    await wait_for_motors(motor1, motor2)

# Sweep profile scan
async def perform_sweep():
    print("  Starting angular sweep of obstacle...")
    await turn_degrees(-30)
    
    angles = []
    distances = []
    
    step_deg = 2
    num_steps = 30
    
    for i in range(num_steps + 1):
        dist = None
        for _ in range(3):
            dist = get_filtered_sonar()
            if dist is not None:
                break
            await asyncio.sleep(0.01)
            
        current_angle = -30 + i * step_deg
        if dist is not None:
            angles.append(current_angle)
            distances.append(dist)
            print(f"    Angle={current_angle} deg, Dist={dist:.1f} cm")
        else:
            print(f"    Angle={current_angle} deg, Dist=Invalid")
            
        if i < num_steps:
            await turn_degrees(step_deg)
            
    await turn_degrees(-30)
    return angles, distances

# Shape analysis helper functions
import math

def filter_sweep_spikes(angles, distances):
    n = len(distances)
    if n < 3:
        return angles, distances
        
    clean_angles = []
    clean_dists = []
    
    if distances[0] is not None:
        clean_angles.append(angles[0])
        clean_dists.append(distances[0])
        
    for i in range(1, n - 1):
        prev_d = distances[i-1]
        curr_d = distances[i]
        next_d = distances[i+1]
        
        if curr_d is None:
            continue
            
        if prev_d is not None and next_d is not None:
            # Filter drop-out spikes (sudden dip)
            if curr_d < prev_d - 12.0 and curr_d < next_d - 12.0:
                print(f"    Filtered sweep dip: Angle={angles[i]} deg, Dist={curr_d:.1f} cm (neighbors: {prev_d:.1f}, {next_d:.1f})")
                continue
            # Filter spike peaks
            if curr_d > prev_d + 12.0 and curr_d > next_d + 12.0:
                print(f"    Filtered sweep peak: Angle={angles[i]} deg, Dist={curr_d:.1f} cm (neighbors: {prev_d:.1f}, {next_d:.1f})")
                continue
                
        clean_angles.append(angles[i])
        clean_dists.append(curr_d)
        
    if distances[-1] is not None:
        clean_angles.append(angles[-1])
        clean_dists.append(distances[-1])
        
    return clean_angles, clean_dists

def fit_line_rmse(x, y):
    n = len(x)
    if n < 3:
        return 0.0
    sum_x = sum(x)
    sum_y = sum(y)
    sum_xx = sum(xi*xi for xi in x)
    sum_yy = sum(yi*yi for yi in y)
    sum_xy = sum(xi*yi for xi, yi in zip(x, y))
    
    denom = (n * sum_xx - sum_x * sum_x)
    if abs(denom) < 1e-6:
        # Near-vertical line
        mean_x = sum_x / n
        return (sum((xi - mean_x)**2 for xi in x) / n)**0.5
        
    m = (n * sum_xy - sum_x * sum_y) / denom
    c = (sum_y - m * sum_x) / n
    
    rss = sum((yi - (m * xi + c))**2 for xi, yi in zip(x, y))
    return (rss / n)**0.5

def analyze_sweep(angles, distances):
    # Filter spikes first
    angles, distances = filter_sweep_spikes(angles, distances)
    
    obstacle_points = []
    for angle, dist in zip(angles, distances):
        if dist is not None and dist < 45.0:
            obstacle_points.append((angle, dist))
            
    if len(obstacle_points) < 8:
        print("    Error: Too few obstacle points detected in scan!")
        return None
        
    # Find the closest point angle to align center
    min_dist = min(p[1] for p in obstacle_points)
    closest_p = [p for p in obstacle_points if p[1] == min_dist][0]
    center_angle = closest_p[0]
    
    print(f"    Closest point: {min_dist:.1f} cm at {center_angle} degrees")
    
    x_all = []
    y_all = []
    rel_angles = []
    
    for angle, dist in obstacle_points:
        rel_angle = angle - center_angle
        rad = math.radians(rel_angle)
        xi = dist * math.sin(rad)
        yi = dist * math.cos(rad)
        x_all.append(xi)
        y_all.append(yi)
        rel_angles.append(rel_angle)
        
    # Define three windows: Left, Right, Center
    left_x, left_y = [], []
    right_x, right_y = [], []
    center_x, center_y = [], []
    
    for xi, yi, ra in zip(x_all, y_all, rel_angles):
        if -24 <= ra <= 0:
            left_x.append(xi)
            left_y.append(yi)
        if 0 <= ra <= 24:
            right_x.append(xi)
            right_y.append(yi)
        if -12 <= ra <= 12:
            center_x.append(xi)
            center_y.append(yi)
            
    rmse_left = fit_line_rmse(left_x, left_y) if len(left_x) >= 4 else 99.0
    rmse_right = fit_line_rmse(right_x, right_y) if len(right_x) >= 4 else 99.0
    rmse_center = fit_line_rmse(center_x, center_y) if len(center_x) >= 4 else 99.0
    
    print(f"    Left RMSE ({len(left_x)} pts): {rmse_left:.3f} cm")
    print(f"    Right RMSE ({len(right_x)} pts): {rmse_right:.3f} cm")
    print(f"    Center RMSE ({len(center_x)} pts): {rmse_center:.3f} cm")
    
    score = min(rmse_left, rmse_right, rmse_center)
    print(f"    Combined Score (min RMSE): {score:.3f} cm")
    return score

async def find_obstacle_direction():
    print("Performing initial 360-degree scan to locate center...")
    angles = []
    distances = []
    
    # 36 steps of 10 degrees = 360 degrees
    step_deg = 10
    num_steps = 36
    
    for i in range(num_steps):
        dist = None
        for _ in range(3):
            dist = get_filtered_sonar()
            if dist is not None:
                break
            await asyncio.sleep(0.01)
            
        current_angle = i * step_deg
        angles.append(current_angle)
        distances.append(dist)
        print(f"  Scan: Angle={current_angle} deg, Dist={dist}")
        
        await turn_degrees(step_deg)
        await asyncio.sleep(0.02)
        
    # Find longest contiguous open sector (dist > 30.0 cm) to identify diagonal pointing to center
    n = len(angles)
    is_open = [d is not None and d > 30.0 for d in distances]
    double_open = is_open + is_open
    
    max_len = 0
    best_start = 0
    current_len = 0
    current_start = 0
    
    for i in range(2 * n):
        if double_open[i]:
            if current_len == 0:
                current_start = i
            current_len += 1
            if current_len > max_len:
                max_len = current_len
                best_start = current_start
        else:
            current_len = 0
            
    mid_index = (best_start + max_len // 2) % n
    target_angle = angles[mid_index]
    
    print(f"Longest open sector starts at index {best_start} with length {max_len}.")
    print(f"Target angle for center of room: {target_angle} degrees.")
    
    # Turn to target_angle
    await turn_degrees(target_angle)
    await asyncio.sleep(0.2)

async def run_mapping_task():
    print("\n================================================")
    print("RUNNING TASK 2: OBSTACLE MAPPING & CLASSIFICATION")
    print("================================================\n")
    
    led_red.value = False
    led_yellow.value = False
    
    # We skip find_obstacle_direction() at startup as it can be buggy and slow.
    # The robot will naturally face the room's open space after bouncing off a corner wall if needed.
    
    while True:
        consecutive_glitches = 0
        
        print("Step 1: Driving forward to locate object...")
        object_found = False
        
        # Start motors driving forward
        motor1.set_rpm(MAX_RPM)
        motor2.set_rpm(MAX_RPM)
        motor1.move_forever(MOVE_DIRECTION)
        motor2.move_forever(MOVE_DIRECTION)
        
        close_count = 0
        loop_count = 0
        
        try:
            while True:
                # 3 samples is fast, responsive, and filters transient spikes
                dist = get_filtered_sonar(samples=3)
                
                if dist is not None:
                    consecutive_glitches = 0
                    
                    # Print log only once every 5 iterations (~0.5s) to avoid spamming the console
                    if loop_count % 5 == 0:
                        print(f"  Sonar: {dist:.1f} cm")
                    loop_count += 1
                    
                    if dist < 33.0:
                        close_count += 1
                        if close_count >= 2:
                            print("  Object detected! Stopping.")
                            object_found = True
                            break
                    else:
                        close_count = 0
                else:
                    consecutive_glitches += 1
                    if consecutive_glitches >= 8:
                        print("  Sonar: Persistent glitch detected (8+ retries). Stopping to recover...")
                        break
                
                await asyncio.sleep(0.05)
        finally:
            # Always ensure motors are stopped when exiting the driving loop
            motor1.stop()
            motor2.stop()
            await wait_for_motors(motor1, motor2)
            
        if not object_found:
            print("  No object detected or glitch recovery triggered. Backing up and turning...")
            await drive_steps(2000, direction=-MOVE_DIRECTION) # Back up ~11 cm
            await turn_degrees(80) # Turn 80 degrees to face a new direction
            continue
            
        # Perform sweep
        angles, dists = await perform_sweep()
        
        valid_dists = [d for d in dists if d is not None]
        if not valid_dists:
            print("  Error: Sweep returned no valid distances. Backing up and turning...")
            await drive_steps(2000, direction=-MOVE_DIRECTION)
            await turn_degrees(100)
            continue
            
        max_d = max(valid_dists)
        print(f"  Sweep max distance: {max_d:.1f} cm")
        
        if max_d < 50.0:
            print("  Detected a WALL. Bouncing...")
            await drive_steps(3500, direction=-MOVE_DIRECTION) # Back up 19 cm
            await turn_degrees(110) # Turn 110 degrees
            await asyncio.sleep(0.2)
        else:
            print("  Detected the OBSTACLE!")
            score1 = analyze_sweep(angles, dists)
            
            print("  Moving closer for Scan 2...")
            await drive_steps(1500) # Move 8 cm closer
            await asyncio.sleep(0.5)
            
            angles2, dists2 = await perform_sweep()
            score2 = analyze_sweep(angles2, dists2)
            
            scores = [s for s in [score1, score2] if s is not None]
            if not scores:
                print("  Error: Both scans failed. Retrying search...")
                await drive_steps(3500, direction=-MOVE_DIRECTION)
                await turn_degrees(110)
                continue
                
            avg_score = sum(scores) / len(scores)
            print(f"\nAverage Shape Score: {avg_score:.3f} cm")
            
            is_triangle = avg_score < 0.18
            if is_triangle:
                print("\n>>> CLASSIFICATION: TRIANGLE <<<")
                led_red.value = True
                led_yellow.value = False
            else:
                print("\n>>> CLASSIFICATION: CIRCLE <<<")
                led_yellow.value = True
                led_red.value = False
                
            print("Task complete. Decision LED is active.")
            break

async def main():
    asyncio.create_task(motor1.run())
    asyncio.create_task(motor2.run())
    await run_mapping_task()
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
