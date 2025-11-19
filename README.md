# 🏎️ RallyCore

## 🌟 Overview
RallyCore is a ROS2- and Nav2-based software stack designed for rally car, which is a rally car for autonomous vehicle development at all kinds of complex terrains. 

All the code was written during my free time after work while I was a Research Assistant at [ZJU FAST Lab](https://github.com/ZJU-FAST-Lab). I am deeply grateful to the wonderful people at ZJU FAST Lab for their support and guidance. Even though this is just a small project, their assistance was invaluable.

## 📦 Installation

I provide an installation guide [here](install.md) 📖, for ROS2 Humble on Ubuntu 22.04.

⚠️ May have some issues with the installation guide, please let me know if you have any questions.

## 🔨 Modification & Customization

### 🛠️ Hardware Details
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

### ⚡ Current-Acceleration Calibration
Motor current to acceleration mapping for precise speed control.

```bash
# 1️⃣ Start hardware
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py

# 2️⃣ Start SLAM
ros2 launch f1tenth_system slam.launch.py

# 3️⃣ Run calibration
ros2 run f1tenth_system current_acc_calib.py
```

📚 **Docs:** [QUICK_START_CALIB.md](QUICK_START_CALIB.md) | [CALIBRATION_WORKFLOW.md](CALIBRATION_WORKFLOW.md)

### 🎯 Pure Pursuit Parameter Tuning
Optimize trajectory tracking before calibration. Default 1 m/s, real-time adjustable.

```bash
# 🚀 Start tuner
ros2 run f1tenth_system pp_param_tuner

# 🎨 GUI tuning
ros2 run rqt_reconfigure rqt_reconfigure
```

**✨ Key features:** 
- 🎮 Manual speed control (0-10 m/s) 
- 🔄 Figure-8 auto-trajectory 
- 📊 Live metrics (CTE/heading RMS)

**⚡ Quick adjust:** `ros2 param set /pp_param_tuner target_speed 2.5`

📚 **Full docs:** `src/f1tenth_system/scripts/readme/`

## 🏗️ Architecture: V1 vs V2

**✨ V2 (Integrated):** RC/Nav2/PP/Calib → `joystick_v2` → VESC  
**📦 V1 (Mux-based):** RC/Nav2 → `joystick` → `ackermann_mux` → VESC

| Feature | V2 ✨ | V1 📦 |
|---------|----|----|
| 🎮 Control nodes | 1️⃣ (joystick_v2) | 2️⃣ (joystick + mux) |
| ⚡ ESC modes | Speed/Current/Duty | Speed only |
| 🔬 Calibration | ✅ Built-in | ❌ Not supported |
| 🐛 Debugging | 😊 Easier | 😅 Complex |

📚 **Full comparison:** `src/f1tenth_system/scripts/readme/`

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













