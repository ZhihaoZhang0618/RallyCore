# Longitudinal Calibration Data Collection (With / Without Localization)

This folder currently provides two longitudinal calibration data-collection approaches:

- **Localization-based (Pure Pursuit Auto Looping)**: requires stable odometry/localization. The vehicle follows a “stadium / figure-8” style trajectory automatically; it sweeps motor current on straights and holds speed on curves. This is suitable for long, repeatable data collection, but typically requires a relatively large open area/track.
- **RC-intervention (Manual Steer, No Localization)**: does not rely on localization. The scripts publish longitudinal commands ("current sweep on straights / speed hold on turns"), while lateral control (steering) is provided by the RC transmitter. This is recommended when odometry drifts (e.g., fastlio/IMU issues) or when you do not have a large enough track.

Both approaches publish the same topic: `/calib/ackermann_cmd` (downstream must parse the `jerk` convention below).

---

## 0.1) Build & Environment Setup (Required)

These scripts are installed as executables via `ament_python`. After the first use or after script updates, rebuild:

```bash
cd ~/RallyCore
colcon build --packages-select f1tenth_system
```

Then source in the **current terminal** (pick the one matching your shell):

```bash
# bash
source install/setup.bash

# zsh
source install/setup.zsh
```

---

## 0.2) `/calib/ackermann_cmd` Convention (Downstream Must Follow)

This repository uses `AckermannDriveStamped.drive.jerk` as a "mode flag" to multiplex speed mode and current mode on the same topic:

- `AckermannDriveStamped.drive.jerk == 2.0`: **current mode**
  - `drive.acceleration` = current (A)
  - `drive.steering_angle` = steering angle (rad)
  - `drive.speed` not used (can be 0)
- `AckermannDriveStamped.drive.jerk == 0.0`: **speed mode**
  - `drive.speed` = target speed (m/s)
  - `drive.steering_angle` = steering angle (rad)
  - `drive.acceleration` can be 0

---

## 1) Localization-based: Pure Pursuit Auto Loop Calibration (Requires `/odom`)

### 1.1 When to Use

- You have stable `nav_msgs/Odometry` (e.g., `/odom` from EKF / fastlio / wheel odometry fusion).
- You want repeatable long-duration collection.
- You have a relatively large open area/track (long straights + safe turn radius). Without space, the loop-based method becomes hard to run safely and cleanly.

### 1.2 Core Scripts

#### (1) `pp_current_acc_calib.py`: Auto loop + segmented current/speed calibration

Script: `src/f1tenth_system/scripts/pp_current_acc_calib.py`

Subscribes:
- `/odom` (`nav_msgs/Odometry`)
- `/vesc/sensors` (`vesc_msgs/VescStateStamped`, telemetry)

Publishes:
- `/calib/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/calib/lookahead_point` (`geometry_msgs/PointStamped`, visualization)

Run:

```bash
ros2 run f1tenth_system pp_current_acc_calib.py
```

Common parameters (examples):

```bash
# calibration mode: acceleration (positive current sweep) / braking (negative-current braking)
ros2 param set /current_acc_calib calibration_mode acceleration

# current ramp slope in acceleration mode (A/s)
ros2 param set /current_acc_calib current_ramp_rate 5.0

# track geometry (start with larger radius/longer straights to reduce lateral disturbance)
ros2 param set /current_acc_calib track_radius 3.0
ros2 param set /current_acc_calib track_straight_length 10.0

# PP parameters (tune with pp_param_tuner first, then copy here)
ros2 param set /current_acc_calib lookahead_gain 1.6
ros2 param set /current_acc_calib min_lookahead 0.3
ros2 param set /current_acc_calib max_lookahead 3.5
ros2 param set /current_acc_calib lateral_error_gain 1.0
ros2 param set /current_acc_calib heading_error_gain 0.4
ros2 param set /current_acc_calib curvature_ff_gain 0.1

# trajectory offsets w.r.t. localization frame (for on-site alignment)
ros2 param set /current_acc_calib traj_offset_x 0.0
ros2 param set /current_acc_calib traj_offset_y 0.0
ros2 param set /current_acc_calib traj_offset_yaw 0.0
```

Logic summary:
- Closed-loop “stadium / figure-8”: straights for calibration, curves for safe speed holding.
- **Straights**: publish `jerk=2.0` (current mode); ramp `drive.acceleration` (A) using `current_ramp_rate`.
- **Curves**: publish `jerk=0.0` (speed mode); hold a tiered target speed.

Suggested rosbag topics:

```bash
ros2 bag record -o pp_calib \
   /odom \
   /vesc/sensors \
   /calib/ackermann_cmd \
   /calib/lookahead_point \
   /tf /tf_static
```

#### (2) `pp_param_tuner.py`: PP parameter tuning helper (optional but strongly recommended)

Script: `src/f1tenth_system/scripts/pp_param_tuner.py`

Note: default interface differs from the calibration node:
- Subscribes: `/odometry/filtered`
- Publishes: `/drive`

If your system uses `/odom` or needs output to `/ackermann_cmd`, use remap/bridge without changing code.

Quick run:

```bash
ros2 run f1tenth_system pp_param_tuner.py --ros-args -p target_speed:=2.0
```

Remap examples:

```bash
# 1) Use /odom instead of /odometry/filtered
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /odometry/filtered:=/odom

# 2) Publish to /ackermann_cmd instead of /drive
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /drive:=/ackermann_cmd
```

---

## 2) RC-intervention: Manual Steer Calibration (No Localization)

Goal: avoid dependence on localization when odometry drifts.

Core logic (cooperating with downstream controller):
- **Straight** (RC steering within deadzone): publish **current mode**, sweep current 0→60A (for calibration)
- **Turning** (RC steering outside deadzone): publish **speed mode**, hold a predefined speed (for safety and to return to the target speed range)
- **Steering**: controlled by RC; straight forces steering=0, turning uses RC steering

Output: `/calib/ackermann_cmd`

---

## 3) Recommended New Workflow: Stage A speed-hold current, then Stage B interval acceleration sweep

When the previous methods do not work well, use this two-stage workflow.

### 3.1 Stage A: hold speeds 1..8 m/s and measure mean phase current

Goal: on straight segments (`steering=0`), measure the mean current required to hold each speed.

Script: `src/f1tenth_system/scripts/speed_hold_current_logger.py`

Run example:

```bash
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
   -p speeds:="[1,2,3,4,5,6,7,8]" \
   -p hold_time_sec:=10.0 \
   -p use_rc_steering:=false \
   -p vesc_topic:=/sensors/core \
   -p output_path:=speed_hold_current_results.txt
```

If you must enable RC steering intervention:

```bash
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

Output:
- `speed_hold_current_results.txt`: mean current per speed (the default strategy is: wait until speed reaches target and remains stable for a while, then sample)
- optional `csv_path`: full time series for debugging

### 3.2 Stage B: for each speed interval v→v+1, sweep current steps to estimate acceleration

Goal: for each interval (1→2, 2→3, …), start near the Stage-A baseline current and increase by `current_step` until `current_max`, measure time-to-reach and estimate acceleration via $a=\Delta v / t$.

Script: `src/f1tenth_system/scripts/speed_interval_accel_sweep.py`

Run example:

```bash
ros2 run f1tenth_system speed_interval_accel_sweep.py --ros-args \
   -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
   -p base_current_file:=speed_hold_current_results.txt \
   -p current_step:=3.0 -p current_max:=80.0 \
   -p vesc_topic:=/sensors/core \
   -p odom_topic:=/odom \
   -p output_path:=speed_interval_accel_results.txt
```

If you must enable RC steering intervention and only run trials on straights:

```bash
ros2 run f1tenth_system speed_interval_accel_sweep.py --ros-args \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

Output:
- `speed_interval_accel_results.txt`: each line is `v0 v1 current_A t_sec accel_mps2 reached`

Notes:
- Before each trial, the script uses **speed mode** to bring the vehicle back to and stabilize at `v0`, then switches to **current mode** to time the acceleration to `v1`.
- Speed feedback comes from `odom.twist.twist.linear.x` (configured by `odom_topic`), and current feedback comes from VESC telemetry (configured by `vesc_topic`).

---

### 3.3 Offline Analysis Prompt (Stage A + B outputs)

Replace the file paths below with your actual generated outputs and send the prompt to your analysis assistant.

```text
You are a vehicle longitudinal dynamics / calibration engineer. Based on the two-stage data collection outputs, build a speed-segmented mapping from motor current to acceleration, and provide a deployable lookup/fit with quality diagnostics.

Input files (replace with real paths):
1) Stage A: speed_hold_current_results.txt
   - Format: speed_mps  mean_current_A  std_current_A  samples
2) Stage B: speed_interval_accel_results.txt
   - Format: v0  v1  current_A  t_sec  accel_mps2  reached
Optional:
- Stage A / Stage B csv_path (if available) for debugging and plotting.

Important collection assumptions:
- No sampling / no trials during turning (steering outside deadzone). After returning straight (inside deadzone), the scripts wait post_turn_settle_sec before resuming sampling/trials.
- Stage B: before each trial, the script stabilizes at v0 in speed mode, then switches to current mode and times acceleration to v1.
- Speed feedback is from /odom.twist.twist.linear.x; current feedback is from VESC telemetry (phase current / iq, etc.).

Tasks:
1) Read & sanity-check
   - Parse both txt files; list Stage-A speed points, range, missing/NaN.
   - Parse all Stage-B trials; compute reached=1 ratio; group by interval (v0->v1) and count samples.
   - Identify obvious outliers: t_sec<=0, invalid accel_mps2, duplicated/disordered current_A, etc.

2) Stage A: baseline current curve
   - Plot I_base(v)=mean_current_A vs v.
   - Smooth/interpolate I_base (e.g., linear interpolation or piecewise fit) and output a queryable function/table.
   - Use std and sample counts to comment on confidence.

3) Stage B: build current→accel model (piecewise by speed interval)
   For each interval (e.g., 1->2, 2->3, ...):
   - Use only reached=1 trials.
   - Scatter plot current_A vs accel_mps2.
   - Provide and compare two modeling options:
     A) Direct fit: a = k*I + b (linear regression)
     B) Subtract baseline: I_eff = I - I_base(v0), fit a = k*I_eff + b or a = k*I_eff.
   - Report k,b, R^2, RMSE, and whether there is nonlinearity/saturation at high current.

4) Deployable output (very important)
   - Generate a monotonic lookup table: for each interval, output a monotonic I->a table (apply monotonic regression if needed).
   - Or output a piecewise-linear parameter table: k,b per interval, with recommended valid ranges.
   - Provide an inversion recipe: given target accel a*, compute the needed current I*.
   - Recommend constraints: I_max, minimum usable current, timeout/unreached handling.

5) Quality diagnostics & next iteration suggestions
   - Which intervals lack data or are too noisy (recommend more repeats or longer reset/stable time).
   - Which current levels frequently timeout (suggest adjusting current_max, trial_timeout_sec, or dv).
   - Recommend practical ranges for post_turn_settle_sec, speed_tolerance, stable_required_sec.

Output requirements:
- List the key plots to generate and what each plot shows.
- Provide the final recommended tables/parameters clearly.
- If interpolation is used, specify method and boundary handling (what to do outside the speed range).
```

---

## IMPORTANT NOTE

This document is currently maintained mainly for the “two-stage workflow” in Section 3.
Content below is supplemental / historical and may not be fully synchronized with code. When in doubt, the code and Section 3 take precedence.

---

## Appendix A) Additional Nodes (Historical / Supplemental)

### A.1 Strict current sweep stages

Script: `src/f1tenth_system/scripts/manual_steer_current_sweep_stages.py`

Run:

```bash
ros2 run f1tenth_system manual_steer_current_sweep_stages.py
```

Common parameters:

```bash
# three stage speeds (m/s)
ros2 param set /manual_steer_current_sweep_stages stage_speeds "[1.5, 3.0, 5.0]"

# current sweep: 0..60A, ramp 5A/s
ros2 param set /manual_steer_current_sweep_stages current_min 0.0
ros2 param set /manual_steer_current_sweep_stages current_max 60.0
ros2 param set /manual_steer_current_sweep_stages current_ramp_rate 5.0

# straight/turning detection and steering mapping (aligned with joystick_control_v2 naming)
ros2 param set /manual_steer_current_sweep_stages steering_channel 4
ros2 param set /manual_steer_current_sweep_stages steering_channel_mid 984
ros2 param set /manual_steer_current_sweep_stages channel_deadzone 100
ros2 param set /manual_steer_current_sweep_stages steering_limit 0.40
ros2 param set /manual_steer_current_sweep_stages steering_reverse true
```

End behavior: after all stages complete, publishes `speed=0` and `current=0` and prints `[DONE]`.

### A.2 Speed-only stages (for link debugging / comparison)

Script: `src/f1tenth_system/scripts/manual_steer_speed_stages.py`

Run:

```bash
ros2 run f1tenth_system manual_steer_speed_stages.py
```

Common parameters:

```bash
# three stage speeds (m/s) and durations (s)
ros2 param set /manual_steer_speed_stages stage_speeds "[1.5, 3.0, 5.0]"
ros2 param set /manual_steer_speed_stages stage_durations "[60.0, 60.0, 60.0]"

# RC topic and output topic
ros2 param set /manual_steer_speed_stages rc_topic /rc/channels
ros2 param set /manual_steer_speed_stages cmd_topic /calib/ackermann_cmd

# steering deadzone and mapping (aligned with joystick_control_v2 naming)
ros2 param set /manual_steer_speed_stages steering_channel 4
ros2 param set /manual_steer_speed_stages steering_channel_mid 984
ros2 param set /manual_steer_speed_stages channel_deadzone 100
ros2 param set /manual_steer_speed_stages steering_limit 0.40
ros2 param set /manual_steer_speed_stages steering_reverse true
```

End behavior: after all stages complete, publishes `speed=0, steering=0` and prints `[DONE]`.

---

## Appendix B) Rosbag Recording (Suggested)

Your current command:

```bash
ros2 bag record -o manual_control1 \
  /odom \
  /drive \
  /imu \
  /livox/imu \
  /sensors/servo_position_command \
  /ackermann_cmd \
  /sensors/core \
  /rc/channels \
  /calib/ackermann_cmd
```

Suggested additional topics (priority order):

1) **Actuator commands (VESC-side)**: to confirm what is actually executed (speed/current/duty) and detect overrides.
- `/commands/motor/speed`
- `/commands/motor/current`
- `/commands/motor/duty_cycle`
- `/commands/servo/position`

2) **TF** (if post-processing needs frame alignment or visualization)
- `/tf`
- `/tf_static`

3) **Clock** (if sim / /clock is used)
- `/clock`

---

## Appendix C) Rosbag Analysis Prompt

Replace `{bag_dir}` with your bag path.

```text
You are a vehicle longitudinal dynamics / calibration engineer. Based on one rosbag recorded during “manual steering intervention + multi-stage speed hold”, build a speed-binned longitudinal model and evaluate the relationship between motor current and acceleration.

Context:
- Upstream publishes /calib/ackermann_cmd (AckermannDriveStamped). In this scheme, drive.jerk=0.0 is used for speed mode.
- When RC steering is within deadzone: treat as straight (steering=0).
- When RC steering is outside deadzone: treat as turning (steering=RC).
- Motor telemetry is from /sensors/core (VescStateStamped), including speed, avg_iq, etc.

Data:
- rosbag directory: {bag_dir}

Please do:
1) Data inspection: list topics, message counts, approximate rates; check timestamps monotonicity and long gaps.
2) Alignment & derived quantities:
   - Cross-check vehicle speed from /sensors/core/state/speed vs /odom.twist.twist.linear.x (explain scaling/latency differences).
   - Compute acceleration from /odom speed (describe differencing, smoothing/low-pass, and any delay compensation).
3) Straight/turning segmentation:
   - Prefer using /rc/channels steering channel with the same deadzone rule as joystick_control_v2.
   - Backup: threshold on /calib/ackermann_cmd.drive.steering_angle.
4) Per-stage statistics:
   - Segment by target speed stages (or cluster by speed bins).
   - For each stage: mean speed, speed variance, mean current (or avg_iq), mean accel, accel noise level.
5) Modeling:
   - Provide at least one deployable model, e.g., a = k(v)*I + b(v) with piecewise-constant k,b by speed bins, or a lookup-table policy.
   - Report R^2 / RMSE and discuss major error sources (turning disturbance, slope, battery voltage, latency).
6) Quality diagnostics:
   - Check contamination: steering input present during supposed “straight” segments.
   - Check saturation, deadzone, or speed-loop oscillation.
7) Next collection suggestions:
   - Minimum straight duration per stage and recommended speed range.
   - Missing topics that would improve analysis.
8) Consider the case where duty cycle saturates at maximum, and propose calibration / control changes for that regime.

Output requirements:
- Provide the final recommended mapping (formula/parameters/table) and clearly specify all filtering and selection rules (thresholds, windows, filter params).
```

---

## Appendix D) How to Hook `/calib/ackermann_cmd` Into Your Stack

These nodes only publish `/calib/ackermann_cmd`. If your control chain uses a different command topic, use launch remap without modifying upstream/downstream code.

Example (remap to the actual command input):

```bash
ros2 run f1tenth_system manual_steer_speed_stages.py \
  --ros-args -r /calib/ackermann_cmd:=/ackermann_cmd
```
