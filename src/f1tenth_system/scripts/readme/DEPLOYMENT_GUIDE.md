# 部署与运维指南

**目标受众**: 运维人员、现场工程师 | **深度**: ⭐⭐⭐

---

## 目录
1. [部署前准备](#部署前准备)
2. [安装和编译](#安装和编译)
3. [首次运行](#首次运行)
4. [常见问题和解决](#常见问题和解决)
5. [监控和日志](#监控和日志)
6. [数据收集](#数据收集)
7. [性能调优](#性能调优)

---

## 部署前准备

### 硬件检查清单

- [ ] **车辆**
  - [ ] 电池已充满 (12V+ for NUC and VESC)
  - [ ] 轮胎气压正确
  - [ ] 前轮对齐 (δ=0时直线)
  - [ ] 转向机构无卡顿

- [ ] **传感器**
  - [ ] LiDAR/激光雷达安装稳固
  - [ ] IMU传感器已安装
  - [ ] 里程计校准完毕
  - [ ] VESC已连接并测试

- [ ] **计算平台**
  - [ ] NUC/Orin已启动
  - [ ] WiFi/网络连接
  - [ ] 存储空间充足 (> 1GB)
  - [ ] 冷却风扇运行

### 软件检查清单

- [ ] ROS2环境
  ```bash
  echo $ROS_DISTRO  # 应该输出: humble
  ```

- [ ] 包编译
  ```bash
  colcon build --packages-select f1tenth_system
  # 应该显示: "Built target f1tenth_system"
  ```

- [ ] 依赖包
  ```bash
  ros2 pkg list | grep -E "(ackermann|vesc|nav2)"
  # 应该列出相关包
  ```

- [ ] Launch文件
  ```bash
  ros2 launch f1tenth_system calib_launch.py --show-args
  # 应该显示所有可用参数
  ```

---

## 安装和编译

### 第1步: 克隆代码

```bash
cd ~/ros2_ws/src
git clone <repository_url> RallyCore
cd RallyCore
```

### 第2步: 安装依赖

```bash
# 安装ROS包依赖
rosdep install --from-paths . --ignore-src -r -y

# 或手动安装关键包
sudo apt-get install -y \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-* \
  python3-pip

# 安装Python依赖
pip install -r src/f1tenth_system/requirements.txt
```

### 第3步: 编译项目

```bash
# 返回工作区根目录
cd ~/ros2_ws

# 编译所有包
colcon build --symlink-install

# 或仅编译f1tenth_system
colcon build --packages-select f1tenth_system --symlink-install
```

**关键标志解释**:
- `--symlink-install`: 允许Python文件编辑后立即生效 (开发模式)
- `--packages-select`: 仅编译特定包 (加快编译)

### 第4步: 验证编译

```bash
# 刷新环境
source install/setup.bash

# 检查包
ros2 pkg list | grep f1tenth_system

# 检查可执行文件
ros2 run f1tenth_system current_acc_calib_node --help
```

---

## 首次运行

### 场景1: 加速标定 (推荐)

```bash
# Terminal 1: 激活ROS环境
source ~/ros2_ws/install/setup.bash

# Terminal 2: 启动EKF本地化 (必须)
ros2 launch f1tenth_system base_orin_bringup.launch.py

# Terminal 3: 启动标定节点 (加速模式)
ros2 launch f1tenth_system calib_launch.py

# Terminal 4: 监控 (可选)
rqt_console  # 查看日志
# 或
ros2 topic echo /calib/ackermann_cmd
```

**预期行为**:
- 绿灯: 节点正在运行
- 黄灯: 等待数据
- 红灯: 错误

**运行时间**: 120秒 (自动)

**输出文件**: `calibration_data.csv`

---

### 场景2: 制动标定

```bash
# 启动制动模式
ros2 launch f1tenth_system calib_launch.py calibration_mode:=braking

# 这会运行负电流 (-5A 到 -20A)
# 时间仍为120秒 (4阶段 × 30秒)
```

**注意**: 确保有足够的停止距离！

---

### 场景3: 自定义参数

```bash
# 增大轨迹半径 (防止轮胎打滑)
ros2 launch f1tenth_system calib_launch.py figure8_radius:=1.8

# 修改车辆质量 (根据实际配置)
ros2 launch f1tenth_system calib_launch.py vehicle_mass:=6.5

# 组合参数
ros2 launch f1tenth_system calib_launch.py \
  calibration_mode:=acceleration \
  figure8_radius:=1.8 \
  vehicle_mass:=6.5 \
  command_frequency:=50
```

---

## 常见问题和解决

### ❌ 错误1: "Could not find package 'f1tenth_system'"

```bash
症状: 
  ros2: error: No package named 'f1tenth_system'

原因:
  工作区未编译或未激活

解决:
  1. 编译工作区
     colcon build --packages-select f1tenth_system
  
  2. 刷新环境
     source ~/ros2_ws/install/setup.bash
  
  3. 验证
     ros2 pkg list | grep f1tenth_system
```

---

### ❌ 错误2: "No topic '/odom' available"

```bash
症状:
  RuntimeError: Could not transform... /odom

原因:
  EKF未运行或里程计发布器未启动

解决:
  1. 启动基础驱动
     ros2 launch f1tenth_system base_orin_bringup.launch.py
  
  2. 验证里程计发布
     ros2 topic echo /odom
     # 应该显示实时位置数据
  
  3. 检查频率
     ros2 topic hz /odom
     # 应该 > 20 Hz
```

---

### ❌ 错误3: "No topic '/vesc/sensors' available"

```bash
症状:
  节点启动但ERPM显示为0

原因:
  VESC驱动未启动或连接错误

解决:
  1. 检查VESC连接
     ls /dev/ttyUSB*  # 应该看到USB设备
  
  2. 手动启动VESC驱动
     ros2 launch vesc_driver vesc_node.launch.py
  
  3. 验证VESC数据
     ros2 topic echo /vesc/sensors
```

---

### ❌ 错误4: "permission denied" (USB设备)

```bash
症状:
  权限拒绝访问 /dev/ttyUSB0

原因:
  用户无USB设备权限

解决:
  1. 添加用户到dialout组
     sudo usermod -a -G dialout $USER
  
  2. 重新登录或运行
     newgrp dialout
  
  3. 重启udev规则
     sudo udevadm control --reload-rules
     sudo udevadm trigger
```

---

### ⚠️ 警告1: "Data collection incomplete"

```bash
症状:
  calibration_data.csv 行数 < 5000

原因:
  1. 运行时间不足120秒
  2. 数据采样丢失
  3. 磁盘空间不足

解决:
  1. 确保完整运行120秒
  2. 检查CPU使用率
     top  # 查看Orin/NUC CPU使用
     # 应该 < 80%
  
  3. 降低控制频率
     ros2 launch f1tenth_system calib_launch.py command_frequency:=25
  
  4. 清理磁盘
     df -h  # 检查空间
     sudo apt-get clean
```

---

### ⚠️ 警告2: 车辆停止或打滑

```bash
症状:
  车不动，或轨迹不稳定

原因:
  1. 8字形半径太小 → 轮子打滑
  2. 目标速度太高 → 控制不稳定
  3. EKF定位漂移

解决:
  1. 增大8字形半径
     ros2 launch f1tenth_system calib_launch.py figure8_radius:=2.0
  
  2. 降低目标速度 (修改代码 CalibrationTier)
     LOW_SPEED: 1.5 → 1.2 m/s
  
  3. 重新运行EKF初始化
     # 手动推动车辆让EKF收敛
```

---

### 💡 提示1: 验证命令是否发送

```bash
# Terminal 1: 启动标定
ros2 launch f1tenth_system calib_launch.py

# Terminal 2: 监视命令输出
ros2 topic echo /calib/ackermann_cmd

# 预期输出:
#   steering_angle: 0.05 (弧度)
#   acceleration: 12.5 (电流)
#   jerk: 1.0 or 2.0
```

---

### 💡 提示2: 查看原始ERPM数据

```bash
# 检查VESC传感器直接读数
ros2 topic echo /vesc/sensors/core

# 输出应该包含:
#   erpm: 4650 (等于1.0 m/s)
#   input_voltage: 12.0
#   temp_mos: 45
```

---

## 监控和日志

### 实时监控

```bash
# 方法1: rqt_console (GUI)
rqt_console

# 方法2: 命令行日志
ros2 run rqt_console rqt_console

# 方法3: 直接查看节点输出
ros2 launch f1tenth_system calib_launch.py  # 输出直接显示
```

### 日志文件位置

```bash
# ROS2日志存储位置
~/.ros/log/

# 查看最新日志
tail -100 ~/.ros/log/*/current_acc_calib*.log

# 搜索错误
grep -i error ~/.ros/log/*/current_acc_calib*.log

# 完整查询
ros2 run rqt_bag rqt_bag  # 可视化数据包
```

### 性能监控

```bash
# CPU/内存使用
top

# 磁盘使用
df -h

# ROS2主题频率
ros2 topic hz /odom
ros2 topic hz /vesc/sensors
ros2 topic hz /calib/ackermann_cmd

# 节点状态
ros2 node list
ros2 node info /current_acc_calib_node
```

---

## 数据收集

### 使用 rosbag2 录制

```bash
# Terminal 1: 启动标定节点
ros2 launch f1tenth_system calib_launch.py

# Terminal 2: 录制所有 topic
rosbag2 record -a -o calibration_run

# 等待120秒，然后Ctrl+C停止
# 生成文件夹: calibration_run/
```

### 从 Bag 提取数据到 CSV

```bash
python3 << 'EOF'
from rosbag2_py import SequentialReader
import pandas as pd

reader = SequentialReader()
reader.open(
    storage_options={'uri': 'calibration_run', 'storage_id': 'sqlite3'},
    converter_options={'input_serialization_format': 'cdr'}
)

data = []
while reader.has_next():
    topic, msg, timestamp = reader.read_next()
    
    if topic == '/odom':
        v = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
        data.append({'timestamp': timestamp/1e9, 'velocity': v})
    elif topic == '/calib/ackermann_cmd' and data:
        data[-1]['current_A'] = msg.drive.acceleration
        data[-1]['steering'] = msg.drive.steering_angle
    elif topic == '/vesc/sensors' and data:
        data[-1]['erpm'] = msg.state.electrical_rpm

df = pd.DataFrame(data)
df.to_csv('calibration_data.csv', index=False)
print(f"提取了 {len(df)} 行数据")
EOF
```

### 数据备份

```bash
# 备份 bag 文件
cp -r calibration_run calibration_run_backup_$(date +%Y%m%d_%H%M%S)

# 远程传输
scp -r calibration_run user@remote:/path/to/store/
```

---

## 性能调优

### 1. 降低CPU使用

```bash
# 方法1: 降低控制频率
ros2 launch f1tenth_system calib_launch.py command_frequency:=25

# 原默认: 50 Hz
# 调整后: 25 Hz
# 影响: 响应延迟 ×2, 数据点减少

# 方法2: 增加采样周期
# (修改代码 publish_rate 参数)
```

### 2. 增加数据精度

```bash
# 方法1: 增加采样频率
ros2 launch f1tenth_system calib_launch.py command_frequency:=100

# 缺点: CPU使用增加 2×

# 方法2: 运行多次标定
# 收集多个 calibration_data_run1.csv
# 合并并平均
```

### 3. 改进稳定性

```bash
# 参数调整
ros2 launch f1tenth_system calib_launch.py \
  figure8_radius:=2.0 \
  lookahead_gain:=2.0 \
  command_frequency:=50
```

| 参数 | 低稳定性 → 高稳定性 |
|------|-------------------|
| `figure8_radius` | 1.4 → 2.0 |
| `lookahead_gain` | 1.0 → 2.0 |
| `command_frequency` | 100 → 25 |

---

## 多次运行和数据管理

### 场景: 收集多个数据集

```bash
# 运行1: 标准配置
ros2 launch f1tenth_system calib_launch.py
cp calibration_data.csv calibration_data_run1.csv

# 运行2: 更高速度
ros2 launch f1tenth_system calib_launch.py vehicle_mass:=6.5
cp calibration_data.csv calibration_data_run2.csv

# 运行3: 更大半径
ros2 launch f1tenth_system calib_launch.py figure8_radius:=2.0
cp calibration_data.csv calibration_data_run3.csv

# 合并数据
python3 << 'EOF'
import pandas as pd

df1 = pd.read_csv('calibration_data_run1.csv')
df2 = pd.read_csv('calibration_data_run2.csv')
df3 = pd.read_csv('calibration_data_run3.csv')

df_combined = pd.concat([df1, df2, df3], ignore_index=True)
df_combined.to_csv('calibration_data_all.csv', index=False)
print(f"合并 {len(df_combined)} 行数据")
EOF
```

---

## 检查清单 - 运行前

```
运行前检查:

□ 编译成功
  colcon build --packages-select f1tenth_system

□ 环境已激活
  source ~/ros2_ws/install/setup.bash

□ 磁盘空间充足
  df -h  # > 1 GB

□ 车辆在安全区域
  (开放停车场或大房间)

□ 电池已充满
  (NUC和VESC都要)

□ 没有障碍物
  (8字形轨迹半径×1.5内)

□ 所有传感器已启动
  ros2 topic list | wc -l  # > 20个topic

□ 基础驱动已运行
  ros2 node list | grep bringup

□ 里程计可用
  ros2 topic echo /odom  # 有数据输出

□ VESC连接良好
  ros2 topic echo /vesc/sensors  # 有ERPM数据
```

---

## 检查清单 - 运行后

```
运行后检查 (120秒后):

□ calibration_data.csv 存在
  ls -lh calibration_data.csv

□ 文件大小合理
  (> 100 KB)

□ 行数完整
  wc -l calibration_data.csv  # > 5000行

□ 数据质量验证
  python3 validate_data.py

□ CSV格式正确
  head -3 calibration_data.csv

□ 备份数据
  cp calibration_data.csv backup/

□ 无报错信息
  grep -i error ~/.ros/log/*/current_acc_calib*.log

□ 性能指标记录
  ros2 topic hz /calib/ackermann_cmd  # > 40 Hz平均
```

---

## 快速命令参考

```bash
# 编译
colcon build --packages-select f1tenth_system

# 激活环境
source ~/ros2_ws/install/setup.bash

# 加速标定 (标准)
ros2 launch f1tenth_system calib_launch.py

# 制动标定
ros2 launch f1tenth_system calib_launch.py calibration_mode:=braking

# 自定义参数
ros2 launch f1tenth_system calib_launch.py figure8_radius:=1.8 vehicle_mass:=6.5

# 监控日志
rqt_console

# 查看命令
ros2 topic echo /calib/ackermann_cmd

# 验证数据
python3 -c "import pandas as pd; df = pd.read_csv('calibration_data.csv'); print(f'行数: {len(df)}')"

# 备份
cp calibration_data.csv calibration_data_$(date +%Y%m%d_%H%M%S).csv
```

---

## 技术支持

| 问题 | 查看章节 |
|------|--------|
| 编译失败 | 安装和编译 |
| 启动失败 | 常见问题 |
| 没有数据 | 数据收集 |
| 数据异常 | 故障排除 (技术指南) |
| 性能差 | 性能调优 |

---

**部署与运维指南完** | 一页纸快速查询 ✓
