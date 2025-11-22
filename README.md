# 🏎️ RallyCore

## 🌟 Overview
RallyCore is a ROS2- and Nav2-based software stack designed for rally car, which is a rally car for autonomous vehicle development at all kinds of complex terrains. 

All the code was written during my free time after work while I was a Research Assistant at [ZJU FAST Lab](https://github.com/ZJU-FAST-Lab). I am deeply grateful to the wonderful people at ZJU FAST Lab for their support and guidance. Even though this is just a small project, their assistance was invaluable.

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
# 🔌 Hardware bringup (V2 - integrated control)
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py

# 🗺️ SLAM/Localization
ros2 launch f1tenth_system slam.launch.py

# 🧭 Navigation
ros2 launch f1tenth_system nav.launch.py
```

### 🏗️ Hardware Bringup Versions
| Version | Launch File | Features |
|---------|-------------|----------|
| **✨ V2 (Recommended)** | `base_orin_livox_bringup_v2.launch.py` | 🎯 Integrated control (joystick_v2) • 🔋 Speed/current/duty modes • 🎨 Simplified architecture |
| 📦 V1 (Legacy) | `base_orin_livox_bringup.launch.py` | 🔀 Separate mux node • 🏛️ Traditional architecture |

**🎉 V2 Advantages:** Single control node • Built-in arbitration • Current control support • Easier debugging

📚 **Architecture details:** See `src/f1tenth_system/scripts/readme/` for V1 vs V2 comparison

## 🔧 Calibration & Tuning

### 🎯 Pure Pursuit Parameter Tuning
**Purpose:** Optimize trajectory tracking before running calibration tests.

```bash
# Start hardware and localization first
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py
ros2 launch f1tenth_system slam.launch.py

# Run PP tuner (default 1 m/s)
ros2 run f1tenth_system pp_param_tuner

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

### ⚡ Current-Acceleration Calibration
**Purpose:** Build motor current to acceleration mapping for precise speed control.

**📋 3-Tier Calibration System:**
| Tier | Duration | Target Speed | Current Range | Purpose |
|------|----------|--------------|---------------|---------|
| 🐌 LOW_SPEED | 0-40s | 1.5 m/s | 5→15 A | Coulomb friction characteristics |
| 📈 MID_SPEED | 40-80s | 3.0 m/s | 8→20 A | Linear drag characteristics |
| ⚡ HIGH_SPEED | 80-120s | 5.0 m/s | 10→25 A | High-speed EMF characteristics |

**🚀 Quick Start:**
```bash
# 1️⃣ Start hardware and SLAM
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py
ros2 launch f1tenth_system slam.launch.py

# 2️⃣ Optional: Record rosbag for later analysis
ros2 bag record -a -o calibration_run

# 3️⃣ Run calibration (auto completes in 120 seconds)
ros2 launch f1tenth_system calib_launch.py

# 4️⃣ Check output
ls calibration_data.csv
```

**⚙️ Configuration Parameters:**
```bash
# Custom trajectory radius (default 1.6m)
ros2 launch f1tenth_system calib_launch.py figure8_radius:=1.8

# Custom vehicle mass (default 6.0kg)
ros2 launch f1tenth_system calib_launch.py vehicle_mass:=6.5

# Braking calibration mode (negative currents)
ros2 launch f1tenth_system calib_launch.py calibration_mode:=braking

# Combined parameters
ros2 launch f1tenth_system calib_launch.py \
  calibration_mode:=acceleration \
  figure8_radius:=1.8 \
  vehicle_mass:=6.5 \
  command_frequency:=50
```

**📊 Data Analysis:**
After calibration, analyze the collected data:
```python
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit

# Load data
df = pd.read_csv('calibration_data.csv')

# Fit linear model: a = k*I + b
def linear(I, k, b):
    return k * I + b

params, _ = curve_fit(linear, df['current_A'], df['estimated_acceleration'])
print(f"Acceleration model: a = {params[0]:.4f}*I + {params[1]:.4f}")

# Calculate fit quality
from sklearn.metrics import r2_score
r2 = r2_score(df['estimated_acceleration'], linear(df['current_A'], *params))
print(f"R² score: {r2:.4f}")
```

**🔍 Troubleshooting:**
- **No odometry:** Check if EKF is running with `ros2 topic echo /odom`
- **Vehicle not moving:** Verify VESC connection with `ros2 topic echo /vesc/sensors`
- **CSV empty:** Ensure full 120s runtime, check disk space with `df -h`
- **Trajectory drift:** Reduce `figure8_radius` or run PP tuner first

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
- 🗺️ [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2.git)

##### 🏛️ All the code was written at FAST Lab, Zhejiang University.

---

## 🚀 Future Work
- 🌄 Add terrain traversability or elevation estimation module
- 🛣️ Add flexible path planning module
- ⚡ Add current&acceleration calibration and control module
- 🎮 Use a better simulation environment, like ISAAC Lab, Autodrive
- 🤖 Use RL to learn end-to-end policies













