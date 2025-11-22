# 🚀 RallyCore Installation Guide

Complete installation guide for ROS2 Humble on Ubuntu 22.04.

---

## 📋 Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [System Setup](#system-setup)
3. [ROS2 Installation](#ros2-installation)
4. [Dependencies Installation](#dependencies-installation)
5. [Workspace Setup](#workspace-setup)
6. [Hardware Configuration](#hardware-configuration)
7. [Verification](#verification)
8. [Troubleshooting](#troubleshooting)

---

## 💻 Hardware Requirements

### 🔍 Pre-Deployment Checklist

#### **🚗 Vehicle Components**
- [ ] 🔋 Battery fully charged (12V+ for NUC and VESC)
- [ ] 🛞 Tire pressure correct
- [ ] ➡️ Front wheels aligned (δ=0 when straight)
- [ ] ⚙️ Steering mechanism moves freely

#### **📡 Sensors**
- [ ] 🔦 Livox Mid-360 LiDAR mounted and connected
- [ ] 🧭 FDILink Deta10 IMU installed (optional)
- [ ] 📏 Odometry calibrated
- [ ] ⚡ VESC connected and tested

#### **💻 Computing Platform**
- [ ] 🖥️ NVIDIA Orin or NUC powered on
- [ ] 📶 WiFi/Network connected
- [ ] 💾 Storage space available (> 10GB recommended)
- [ ] 🌬️ Cooling fan operational

#### **🎮 Remote Control**
- [ ] 📻 RadioMaster Pocket ELRS configured
- [ ] 🔗 Receiver paired and bound
- [ ] 🔌 Connected to computing platform via USB

---

## 🛠️ System Setup

### Step 1: Basic Tools Installation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install net-tools openssh-server curl wget git vim -y

# Start SSH service
sudo service ssh start
sudo service ssh status

# Check network
ifconfig
```

### Step 2: Remote Desktop (Optional)

```bash
# NoMachine for x86 platforms
wget https://download.nomachine.com/download/8.13/Linux/nomachine_8.13.1_1_amd64.deb
sudo dpkg -i nomachine_8.13.1_1_amd64.deb
```

### Step 3: Terminal Enhancement (Optional)

```bash
# Install Zsh and Oh-My-Zsh
sudo apt install zsh terminator -y
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Install plugins
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions

# Edit ~/.zshrc and add:
# plugins=(git zsh-autosuggestions zsh-syntax-highlighting)
# setopt no_nomatch
```

---

## 🤖 ROS2 Installation

### Method 1: Official Installation (International)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Method 2: FishROS Installation (China)

```bash
# One-line installation script
wget http://fishros.com/install -O fishros && bash fishros

# Follow prompts to select:
# - ROS2 Humble
# - Desktop installation
```

### Post-Installation Configuration

```bash
# Add to ~/.bashrc or ~/.zshrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "eval \"\$(register-python-argcomplete3 ros2)\"" >> ~/.bashrc
echo "eval \"\$(register-python-argcomplete3 colcon)\"" >> ~/.bashrc

# Configure RMW (Optional but recommended)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Source changes
source ~/.bashrc
```

---

## 📦 Dependencies Installation

### Core C++ Libraries

```bash
# CppLinuxSerial (for serial communication)
git clone https://github.com/gbmhunter/CppLinuxSerial.git
cd CppLinuxSerial
mkdir build && cd build
cmake .. && make
sudo make install

# Serial ROS2 wrapper
git clone https://github.com/RoverRobotics-forks/serial-ros2.git
cd serial-ros2
mkdir build && cd build
cmake .. && make
sudo make install
```

### System Libraries

```bash
sudo apt install -y \
    ros-humble-asio-cmake-module \
    libasio-dev \
    rapidjson-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev
```

### ROS2 Packages

```bash
# Enable bash wildcard expansion for zsh users
setopt no_nomatch

# Install ROS2 packages (all at once)
sudo apt install -y \
    ros-humble-udp-msgs* \
    ros-humble-ackermann-* \
    ros-humble-plotjuggler* \
    ros-humble-diagnostic-* \
    ros-humble-robot-localization* \
    ros-humble-rmw-cyclonedds-* \
    ros-humble-slam-toolbox* \
    ros-humble-rqt* \
    ros-humble-nav2* \
    ros-humble-tf* \
    ros-humble-map* \
    ros-humble-pcl-*
```

### Python Dependencies

```bash
sudo apt install -y python3-pip python3-numpy python3-matplotlib
pip3 install pandas scipy scikit-learn
```

---

## 🏗️ Workspace Setup

### Step 1: Create Workspace Structure

```bash
# Clone RallyCore
git clone --recursive git@github.com:ZhihaoZhang0618/RallyCore.git

# Update submodules
cd RallyCore
git submodule update --init --recursive
```

### Step 2: Install Livox Driver

```bash
# Create separate workspace for Livox
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src

# Clone Livox ROS2 driver
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# Follow official build instructions
cd ~/livox_ws
colcon build

# Add to shell config
echo "source ~/livox_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: Build RallyCore

```bash
cd ~/RallyCore

# Build all packages
colcon build --symlink-install

# Or build selectively (faster)
colcon build --packages-select f1tenth_system --symlink-install

# Source workspace
echo "source ~/RallyCore/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Build Flags Explanation:**
- `--symlink-install`: Python files take effect immediately without rebuild
- `--packages-select`: Build only specified packages

---

## ⚙️ Hardware Configuration

### USB Device Rules

```bash
# Copy udev rules for consistent device naming
cd ~/RallyCore
sudo cp rulesForNUC/*.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Remove conflicting brltty (if present)
sudo apt remove brltty
```

**Rule Files:**
- `ch340ForELRS.rules` - ELRS receiver (RC remote)
- `cp2102ForFDIMU.rules` - FDILink IMU
- `vesc.rules` - VESC motor controller

### Network Configuration

```bash
# Configure wired ethernet for Livox LiDAR
# Set static IP via Network Manager or netplan

# Example static IP:
# IP: 192.168.1.5
# Netmask: 255.255.255.0
# Gateway: 192.168.1.1

# For Livox Mid-360, LiDAR IP should be: 192.168.1.1xx
```

### CycloneDDS Configuration (Optional)

```bash
# Create CycloneDDS config file
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
EOF

# Add to shell config
echo "export CYCLONEDDS_URI=~/cyclonedds.xml" >> ~/.bashrc
```

---

## ✅ Verification

### Check ROS2 Installation

```bash
# Verify ROS2 version
echo $ROS_DISTRO  # Should output: humble

# Check available packages
ros2 pkg list | grep f1tenth_system

# Test topic listing
ros2 topic list
```

### Check Workspace Build

```bash
# Verify package installation
ros2 pkg list | grep -E "(f1tenth|vesc|ackermann|livox)"

# Check executables
ros2 run f1tenth_system pp_param_tuner --help
ros2 run f1tenth_system current_acc_calib --help

# Verify launch files
ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py --show-args
```

### Test Hardware Connections

```bash
# Check VESC connection
ros2 topic echo /vesc/sensors

# Check IMU (if installed)
ros2 topic echo /imu/data

# Check RC receiver
ros2 topic echo /joy

# Check Livox LiDAR
ros2 topic echo /livox/lidar
```

---

## 🐛 Troubleshooting

### ❌ Error: "Could not find package 'f1tenth_system'"

**Cause:** Workspace not built or sourced

**Solution:**
```bash
cd ~/RallyCore
colcon build --packages-select f1tenth_system
source install/setup.bash
```

### ❌ Error: "No module named 'rclpy'"

**Cause:** ROS2 environment not sourced

**Solution:**
```bash
source /opt/ros/humble/setup.bash
source ~/RallyCore/install/setup.bash
```

### ❌ Error: USB device not found (VESC/IMU/RC)

**Cause:** Incorrect permissions or udev rules not applied

**Solution:**
```bash
# Check device connection
ls /dev/ttyUSB* /dev/ttyACM*

# Re-apply udev rules
sudo cp ~/RallyCore/rulesForNUC/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -aG dialout $USER
# Logout and login again for changes to take effect
```

### ❌ Error: Livox LiDAR not detected

**Cause:** Network configuration incorrect

**Solution:**
```bash
# Verify network interface
ifconfig

# Ping LiDAR
ping 192.168.1.1xx  # Replace xx with your LiDAR ID

# Check Livox config
cat ~/livox_ws/src/livox_ros_driver2/config/MID360_config.json

# Verify IP matches your network setup
```

### ❌ Error: Build fails with "Could not find..."

**Cause:** Missing dependencies

**Solution:**
```bash
# Install missing dependencies
cd ~/RallyCore
rosdep install --from-paths src --ignore-src -r -y

# Or manually install specific package
sudo apt install ros-humble-<package-name>
```

### ❌ Error: "colcon: command not found"

**Cause:** colcon not installed

**Solution:**
```bash
sudo apt install python3-colcon-common-extensions
```

### 🔍 General Debugging Commands

```bash
# Check ROS2 daemon status
ros2 daemon status

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start

# View all nodes
ros2 node list

# View node details
ros2 node info /<node_name>

# Check topic data rate
ros2 topic hz /<topic_name>

# Monitor system logs
journalctl -f

# ROS2 logs location
ls ~/.ros/log/
```

---

## 🎯 Shell Aliases (Optional)

Add these to `~/.bashrc` or `~/.zshrc` for convenience:

```bash
# ROS2 environment
alias source_ros='source /opt/ros/humble/setup.bash'
alias source_ws='source ~/RallyCore/install/setup.bash'
alias source_livox='source ~/livox_ws/install/setup.bash'

# Launch shortcuts
alias base_v2='ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py'
alias base_v1='ros2 launch f1tenth_system base_orin_livox_bringup.launch.py'
alias slam='ros2 launch f1tenth_system slam.launch.py'
alias nav='ros2 launch f1tenth_system nav.launch.py'
alias localization='ros2 launch f1tenth_system localization_slam.launch.py'

# Calibration shortcuts
alias pp_tune='ros2 run f1tenth_system pp_param_tuner'
alias calib='ros2 launch f1tenth_system calib_launch.py'

# Build shortcuts
alias build_ws='cd ~/RallyCore && colcon build --symlink-install'
alias build_f1='cd ~/RallyCore && colcon build --packages-select f1tenth_system --symlink-install'

# Topic monitoring
alias topics='ros2 topic list'
alias nodes='ros2 node list'
alias check_odom='ros2 topic echo /odom'
alias check_vesc='ros2 topic echo /vesc/sensors'
```

After adding aliases, reload your shell config:
```bash
source ~/.bashrc  # or source ~/.zshrc
```

---

## 📝 Next Steps

After successful installation:

1. **✅ Verify all hardware connections** - Follow the checklist above
2. **🚗 Test basic bringup** - Run `ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py`
3. **🗺️ Test SLAM** - Run `ros2 launch f1tenth_system slam.launch.py` and drive around
4. **🎯 Run PP tuner** - Optimize trajectory tracking with `ros2 run f1tenth_system pp_param_tuner`
5. **⚡ Run calibration** - Collect motor data with `ros2 launch f1tenth_system calib_launch.py`
6. **🧭 Test navigation** - Run autonomous navigation with `ros2 launch f1tenth_system nav.launch.py`

Refer to the main [README.md](README.md) for detailed usage instructions.

---

**Installation complete! 🎉** You're now ready to use RallyCore.
