import board
import digitalio
import time
import asyncio
import analogio

from motor import StepperMotor

# RPM Configuration (rescaling analog values 0-65535 to RPM range)
MIN_RPM = 5.0   # User suggested 5 RPM is a good minimum speed
MAX_RPM = 10.0  # Keep motor at 10 RPM max

# Calibration parameters
CALIBRATION_STEPS = 6144  # Number of steps for a 360-degree rotation (approx. 3 wheel revs)
THRESHOLD_FRACTION = 0.2  # Threshold above/below ambient to trigger light following (20% of range)
LIGHT_POLARITY = 1        # 1 if more light increases sensor value (e.g. photodiode), -1 if it decreases


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

# Calibration state
min_left, max_left = 65535, 0
min_right, max_right = 65535, 0
threshold_left = 0.0
threshold_right = 0.0
calibrated = False

async def calibrate_sensors():
    global min_left, max_left, min_right, max_right
    global threshold_left, threshold_right, calibrated
    
    print("Starting calibration... Spinning 360 degrees to scan ambient light.")
    
    # Spin in place: Motor 1 forward, Motor 2 reverse
    motor1.set_rpm(8.0)
    motor2.set_rpm(8.0)
    motor1.move(CALIBRATION_STEPS)
    motor2.move(-CALIBRATION_STEPS)
    
    min_l, max_l = 65535, 0
    min_r, max_r = 65535, 0
    
    while motor1.busy or motor2.busy:
        left_val = light_sensor_left.value
        right_val = light_sensor_right.value
        
        if left_val < min_l: min_l = left_val
        if left_val > max_l: max_l = left_val
        if right_val < min_r: min_r = right_val
        if right_val > max_r: max_r = right_val
        
        print(f"Scanning... Left: {left_val}, Right: {right_val}")
        await asyncio.sleep(0.05)
        
    min_left, max_left = min_l, max_l
    min_right, max_right = min_r, max_r
    
    # Calculate thresholds based on polarity
    range_l = max_left - min_left
    range_r = max_right - min_right
    
    if LIGHT_POLARITY == 1:
        threshold_left = min_left + range_l * THRESHOLD_FRACTION
        threshold_right = min_right + range_r * THRESHOLD_FRACTION
    else:
        threshold_left = max_left - range_l * THRESHOLD_FRACTION
        threshold_right = max_right - range_r * THRESHOLD_FRACTION
        
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
            motor1.move_forever(1)
    else:
        if motor1.run_forever:
            motor1.stop()
        rpm1 = 0.0
        
    if light_detected_r:
        rpm2 = max(MIN_RPM, min(MAX_RPM, rpm2))
        motor2.set_rpm(rpm2)
        if not motor2.run_forever:
            motor2.move_forever(1)
    else:
        if motor2.run_forever:
            motor2.stop()
        rpm2 = 0.0

    return rpm1, rpm2

#Loop for reading sensor values and updating motor speed
async def read_sensors():
    while True:
        if not calibrated:
            await asyncio.sleep(0.1)
            continue
            
        # Read raw light values (0-65535)
        left_val = light_sensor_left.value
        right_val = light_sensor_right.value
        
        rpm1, rpm2 = update_motor_behaviors(left_val, right_val)
        
        print(f"Sensors: L={left_val} (RPM={rpm1:.2f}), R={right_val} (RPM={rpm2:.2f})")
            
        await asyncio.sleep(0.05)


async def main():
    asyncio.create_task(motor1.run())
    asyncio.create_task(motor2.run())
    # Start calibration
    await calibrate_sensors()
    # After calibration, start reading sensors
    asyncio.create_task(read_sensors())
    # Wait on demo
    await demo()

if __name__ == "__main__":
    asyncio.run(main())
