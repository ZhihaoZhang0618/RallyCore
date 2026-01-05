# 🏎️ RallyCore

## 🌟 Overview
RallyCore is a ROS2- and Nav2-based software stack designed for rally car, which is a rally car for autonomous vehicle development at all kinds of complex terrains. 

The hardware setup and basic software framework were developed during my time as a Research Assistant at [ZJU FAST Lab](https://github.com/ZJU-FAST-Lab). I am deeply grateful to the wonderful people at ZJU FAST Lab for their invaluable support and guidance. Currently, I am pursuing my MPhil at PolyU, working in the AIMS Lab. Advanced algorithm development and features will be gradually implemented during my spare time throughout my graduate studies. Stay tuned for more updates!

## 📦 Installation

I provide an installation guide [here](install.md) 📖, for ROS2 Humble on Ubuntu 22.04.

⚠️ May have some issues with the installation guide, please let me know if you have any questions.

## 🔨 Modification & Customization

### 🛠️ Hardware Details
**💻 Computing Platform:** NVIDIA Orin or NUC (recommended)

**🔦 Lidar:** mid360 (main sensor)

**📷 Camera:** Coming soon...

**🧭 IMU:** fdilink Deta10 (optional, yaw estimation)

**🎮 Remote Controller:** RadioMaster Pocket ELRS version (much better than XBOX series controller) 

📝 **For more hardware details:** Coming soon.

### ⚙️ VESC Interface
The modified VESC interface is based on the VESC interface provided by Veddar VESC Interface. 

**Modifications:**
- ✨ Fixed odometry computation to eliminate speed delay when decelerating from high velocities to stop

### 🎛️ ackermann_mux
- ✅ Added scripts to process messages from ELRS driver and publish to `/teleop`

### 🗺️ Nav2 Parameters
- 📚 Configured based on [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)

## 🚀 Main Launch Files

### ⚡ Quick Start (Recommended)
```bash
# 🔌 Hardware bringup (V3 - Point-LIO, Latest)
ros2 launch f1tenth_system base_orin_livox_bringup_v3.launch.py

# Alternative: V2 - FAST-LIO2 with EKF fusion
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py

# 🗺️ SLAM/Localization
ros2 launch f1tenth_system slam.launch.py

# 🧭 Navigation
ros2 launch f1tenth_system nav.launch.py
```

### 🏗️ Hardware Bringup Versions
| Version | Launch File | LIO Backend | Features |
|---------|-------------|-------------|----------|
| **🚀 V3 (Latest)** | `base_orin_livox_bringup_v3.launch.py` | **Point-LIO** | 🎯 Point-LIO odometry • 🚫 No EKF fusion • ⚠️ **Accuracy not stable** |
| **✨ V2** | `base_orin_livox_bringup_v2.launch.py` | **FAST-LIO2** | 🎯 Integrated control (joystick_v2) • 🔋 Speed/current/duty modes • 🤖 EKF fusion |
| 📦 V1 (Legacy) | `base_orin_livox_bringup.launch.py` | **FAST-LIO2** | 🔀 Separate mux node • 🏛️ Traditional architecture • 🤖 EKF fusion |

**⚠️ V3 Status:** Point-LIO integrates well with mid360 structurally, but odometry drift/accuracy performance is currently **below expectations**. Recommend **V2 (FAST-LIO2)** for reliable localization and calibration work.
**⚙️ V2 Advantages:** Single control node • Built-in arbitration • Current control support • Easier debugging • Proven odometry accuracy

📚 **Calibration docs (maintained):** [src/f1tenth_system/scripts/README_EN.md](src/f1tenth_system/scripts/README_EN.md)

## 🔧 Calibration & Tuning

✅ The longitudinal calibration workflow is documented and maintained in:

- [src/f1tenth_system/scripts/README_EN.md](src/f1tenth_system/scripts/README_EN.md)

It includes:
- `/calib/ackermann_cmd` jerk convention (speed/current)
- Localization-based (Pure Pursuit) and RC-intervention (Manual Steer) data collection
- Recommended **two-stage workflow** (Stage A speed-hold current + Stage B interval accel sweep)

### 🎯 Pure Pursuit Parameter Tuning
**Purpose:** Optimize trajectory tracking before running calibration tests.

```bash
# Start hardware and localization first
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py
ros2 launch f1tenth_system slam.launch.py

# Run PP tuner (default 1 m/s)
ros2 run f1tenth_system pp_param_tuner.py

# Real-time speed adjustment
ros2 param set /pp_param_tuner target_speed 2.5

# GUI tuning interface
ros2 run rqt_reconfigure rqt_reconfigure
```

**✨ Key Features:**
- 🎮 Manual speed control (0-10 m/s)
- 🔄 Automatic Figure-8 trajectory generation
- 📊 Live metrics: CTE RMS, heading RMS
- ⚙️ Dynamic parameter tuning: `wheelbase`, `lookahead_gain`, `min_lookahead`, `max_lookahead`

**🎯 Tuning Tips:**
- Start with low speed (1-2 m/s) to verify trajectory tracking
- Increase `lookahead_gain` if path is too aggressive
- Decrease `lookahead_gain` if path tracking is too loose
- Check RViz visualization: `/pp/current_trajectory` and `/pp/lookahead_point`

---

### ⚡ Longitudinal Calibration (Recommended)

#### Option A: Localization-based (Pure Pursuit)

This option requires stable odometry and a relatively large open area/track (long straights and safe turn radius), because the vehicle needs enough space to run repeatable loops.

See [src/f1tenth_system/scripts/README_EN.md](src/f1tenth_system/scripts/README_EN.md) → Localization-based (Pure Pursuit).

```bash
ros2 run f1tenth_system pp_current_acc_calib.py
```

#### Option B (Recommended): RC-intervention (No localization) — Two-stage workflow

This is the recommended workflow when odometry/localization is unstable, or when you do not have a large enough calibration track.

Stage A (speed hold → mean current):

```bash
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
    -p speeds:="[1,2,3,4,5,6,7,8]" \
    -p hold_time_sec:=10.0 \
    -p vesc_topic:=/sensors/core \
    -p output_path:=speed_hold_current_results.txt
```

Stage B (per speed interval → accel vs current, speed from `/odom`):

```bash
ros2 run f1tenth_system speed_interval_accel_sweep.py --ros-args \
    -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
    -p base_current_file:=speed_hold_current_results.txt \
    -p current_step:=3.0 -p current_max:=80.0 \
    -p vesc_topic:=/sensors/core \
    -p odom_topic:=/odom \
    -p output_path:=speed_interval_accel_results.txt
```

If you need RC steering intervention during data collection, enable it and the scripts will:
- NOT sample / NOT run trials during turning (out of deadzone)
- Wait `post_turn_settle_sec` after returning straight before resuming

```bash
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
    -p use_rc_steering:=true -p rc_topic:=/rc/channels -p rc_timeout_sec:=0.25 \
    -p post_turn_settle_sec:=0.8

ros2 run f1tenth_system speed_interval_accel_sweep.py --ros-args \
    -p use_rc_steering:=true -p rc_topic:=/rc/channels -p rc_timeout_sec:=0.25 \
    -p post_turn_settle_sec:=0.8
```

📊 Analysis prompt templates live in: [src/f1tenth_system/scripts/README_EN.md](src/f1tenth_system/scripts/README_EN.md).

## 🏗️ Architecture: V1 vs V2

### 📐 System Architecture Diagrams

**✨ V2 Architecture (Integrated Control):**
```
┌─────────────┐
│  RC Remote  │──┐
└─────────────┘  │
┌─────────────┐  │
│ Nav2 /drive │──┤
└─────────────┘  │    ┌──────────────────┐    ┌──────────┐
┌─────────────┐  ├───▶│ joystick_v2.py   │───▶│   VESC   │
│  PP Tuner   │──┤    │ (Integrated Mux) │    │  Driver  │
└─────────────┘  │    └──────────────────┘    └──────────┘
┌─────────────┐  │
│ Calibration │──┘
└─────────────┘
```

**📦 V1 Architecture (Mux-based):**
```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────┐
│  RC Remote  │───▶│ joystick.py  │───▶│  ackermann  │───▶│   VESC   │
└─────────────┘    └──────────────┘    │     mux     │    │  Driver  │
┌─────────────┐                        │  (Priority) │    └──────────┘
│ Nav2 /drive │───────────────────────▶│             │
└─────────────┘                        └─────────────┘
```

### 🔄 Feature Comparison

| Feature | V2 ✨ (Recommended) | V1 📦 (Legacy) |
|---------|----|----|
| 🎮 **Control Nodes** | 1 (joystick_v2) | 2 (joystick + mux) |
| ⚡ **ESC Modes** | Speed/Current/Duty | Speed only |
| 🔬 **Calibration Support** | ✅ Built-in `/calib/*` | ❌ Not supported |
| 🎚️ **Command Arbitration** | Built-in logic | External mux node |
| 🐛 **Debugging** | Easier (single node) | Complex (multiple nodes) |
| 📡 **Input Topics** | `/drive`, `/calib/ackermann_cmd` | `/teleop`, `/drive` |
| 📤 **Output Topic** | `/ackermann_drive` | `/ackermann_drive` |
| 🎛️ **Mode Switching** | RC channel 10 | YAML config |

### 🎯 When to Use Each Version

**Use V2 if:**
- ✅ Running calibration experiments
- ✅ Need current/duty control modes
- ✅ Want simplified debugging
- ✅ Prefer integrated control logic

**Use V1 if:**
- 📦 Only need Nav2 navigation (speed control)
- 📦 Require strict priority-based arbitration
- 📦 Legacy system compatibility

### 📊 V2 Control Flow Details

```python
# joystick_v2.py simplified logic
def control_logic():
    if rc_channel_10 == HIGH:
        mode = "teleop"  # RC manual control
    elif calibration_active:
        mode = "calib"   # Calibration mode
    else:
        mode = "nav"     # Navigation mode
    
    if esc_mode == "CURRENT":
        command.current = target_current  # For calibration
    elif esc_mode == "SPEED":
        command.speed = target_speed      # For navigation
    elif esc_mode == "DUTY":
        command.duty_cycle = target_duty  # For advanced control
```

### 🔌 ROS2 Topic Interface

**V2 Subscribed Topics:**
- `/drive` (AckermannDriveStamped) - Navigation commands
- `/calib/ackermann_cmd` (AckermannDriveStamped) - Calibration commands
- `/joy` (Joy) - RC remote input

**V2 Published Topics:**
- `/ackermann_drive` (AckermannDriveStamped) - Final command to VESC

**Message Fields Usage:**
```yaml
AckermannDriveStamped:
  drive:
    steering_angle: float    # Steering angle in radians
    speed: float             # Target speed (m/s) for SPEED mode
    acceleration: float      # Current (A) for CURRENT mode
    jerk: float             # Mode flag: 0=teleop, 1=curve, 2=straight
    steering_angle_velocity: # Unused
```

---

## 🙏 Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:

- 🏎️ [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- 🏁 [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)
- 🎯 [f1tenth_system](https://github.com/f1tenth/f1tenth_system)
- 📡 [ros2_crsf_receiver](https://github.com/AndreyTulyakov/ros2_crsf_receiver.git)
- 🔀 [ackermann_mux](https://github.com/z1047941150/ackermann_mux.git)
- ⚡ [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- 🗺️ [FAST-LIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2.git)
- 🎯 [Point-LIO_ROS2](https://github.com/dfloreaa/point_lio_ros2.git)

##### 🏛️ Hardware and basic software were developed at FAST Lab, Zhejiang University.
##### 🎓 Currently pursuing MPhil at PolyU AIMS Lab, with ongoing development in progress.

---

## 🚀 Future Work
- 🌄 Add terrain traversability or elevation estimation module
- 🛣️ Add flexible path planning module
- ✅ ~~Add current&acceleration calibration and control module~~ (Completed ✨)
- 🎮 Use a better simulation environment, like ISAAC Lab, Autodrive
- 🤖 Use RL to learn end-to-end policies
- 🗺️ Integrate additional LIO backends (LVI-SAM, DLIO, etc.)













