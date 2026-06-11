# Simple Bot Firmware

This directory contains the CircuitPython firmware for the robot.

## Hardware
- **Microcontroller**: Raspberry Pi Pico (W)
- **Motors**: 2 x 28BYJ-48 Stepper Motors (controlled via ULN2003 drivers)
- **Pin Mapping**:
  - Motor 1: GP15, GP16, GP17, GP18
  - Motor 2: GP19, GP20, GP21, GP22

## Libraries
The libraries in the `lib` folder are compiled `.mpy` files for CircuitPython.

To manage dependencies, you can use `circup`:
```bash
circup install -r requirements.txt
```

Required libraries:
- `adafruit_motor`
- `asyncio`
- `adafruit_ticks`

## Files
- `code.py`: Main entry point and demo sequence.
- `motor.py`: Stepper motor control class using `asyncio`.
- `tests/test_smoke.py`: Host-side smoke tests verifying syntax, constraints, and control logic using python's built-in `unittest`.

## Running Host-Side Tests
To verify code changes (syntax check, pins, control loop logic, math) on a host computer before deploying to the Raspberry Pi Pico, run:
```bash
PYTHONPATH=.:typings python3 -m unittest tests/test_smoke.py
```

