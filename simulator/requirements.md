# Docking Simulator Requirements

## Purpose

The simulator has two main goals:

1. **Verify that a docking algorithm works in ideal conditions.**
2. **Add noise and error to test whether the algorithm can self-correct.**

The simulator does not need to perfectly model real hardware at first. It should be simple, configurable, and useful for comparing strategies.

---

## Competition Task Summary

The robot starts:

- at least **100 cm** from the target
- at a random position
- at a random orientation

The robot must autonomously dock over the target mat.

A dock is successful when:

- the robot is stationary
- the termination indicator has been triggered
- the target is fully covered by the robot base

Important rule:

> The termination indicator must not trigger while the robot is moving.

---

## Scoring Model

At termination:

| Condition | Score |
|---|---:|
| Target is 100% covered by robot base | 10 |
| Target is partially covered | 2.5 |
| Target was touched earlier, but not covered at termination | 1 |
| Indicator triggered while robot was moving | -5 |

The simulator should calculate:

```text
coverage_ratio = covered_target_area / total_target_area
```

Where:

```text
coverage_ratio = 1.0  => full coverage
coverage_ratio > 0    => partial coverage
coverage_ratio = 0    => no coverage
```

---

## Robot Geometry

Use the current robot top-view design.

Default values:

```text
robot_width_cm  = 12
robot_length_cm = 16
```

The robot footprint may be approximated as either:

1. a simple rectangle, or
2. the chamfered SVG-like polygon

For early simulation, a rectangle is acceptable.

Later, use the chamfered polygon for more accurate coverage checking.

---

## Target Geometry

The target mat must be configurable.

Required variables:

```text
target_width_cm  = 1 to 9
target_height_cm = 1 to 9
```

The simulator should allow rectangular targets, not only squares.

Examples:

```text
1 × 1 cm
3 × 5 cm
5 × 5 cm
9 × 9 cm
```

Optional but recommended:

```text
target_rotation_deg = configurable
```

This allows testing whether rotated rectangular targets are harder to fully cover.

---

## Start Conditions

Required variables:

```text
start_distance_min_cm = 100
start_distance_max_cm = configurable
start_position_random = true
robot_orientation_random = true
```

Each test run should begin from a randomized pose:

```text
robot_x_cm
robot_y_cm
robot_heading_deg
```

The target may initially be placed at the origin:

```text
target_x_cm = 0
target_y_cm = 0
```

---

## Motion Model

The simulator should support simple robot movement:

```text
forward_speed_cm_s
turn_speed_deg_s
time_step_s
```

At minimum, support commands:

```text
move_forward
turn_left
turn_right
stop
```

Optional later:

```text
differential_drive_left_speed
differential_drive_right_speed
```

---

## Noise and Error Variables

The simulator should make errors configurable.

Motion error:

```text
position_noise_cm
heading_noise_deg
forward_slip_percent
turn_error_percent
stop_delay_s
```

Sensor error:

```text
sensor_false_positive_rate
sensor_false_negative_rate
sensor_range_noise_cm
sensor_angle_noise_deg
```

The simulator should be able to run with:

```text
noise_enabled = false
```

for ideal testing, and:

```text
noise_enabled = true
```

for robustness testing.

---

## Sensor Model

Start simple.

Minimum virtual sensors:

```text
target_visible        # true/false
target_angle_deg      # relative angle to target
target_distance_cm    # approximate distance
target_under_robot    # true/false
coverage_ratio        # actual geometric coverage, for scoring/debug
```

Later, we can replace these with more realistic models:

```text
camera_cone_sensor
downward_color_sensor
hall_sensor
imu_stillness_sensor
```

---

## Algorithm Testing

The simulator should allow testing multiple simple algorithms first.

Baseline algorithms:

1. **Straight walk**
2. **Random walk**
3. **Rotate-scan then drive**
4. **Expanding spiral or expanding square**
5. **Detect, approach, slow final correction**

The goal is to establish simple baselines before adding smarter behavior.

---

## Run Modes

The simulator should support:

### Single Run

Shows one animated run.

Useful for debugging.

### Batch Run

Runs many trials without animation.

Useful for statistics.

Required batch metrics:

```text
success_rate_full_coverage
partial_coverage_rate
touched_but_failed_rate
moving_termination_failures
average_score
average_time_s
```

---

## Main Design Principle

The simulator should expose failure cases.

It should help answer:

- Does the algorithm find the target?
- Does it fully cover the target?
- Does it stop before triggering the indicator?
- How sensitive is success to target size?
- How sensitive is success to movement noise?
- Can the algorithm recover from bad alignment?

The simulator should prioritize clear logic and useful debugging over visual polish.
