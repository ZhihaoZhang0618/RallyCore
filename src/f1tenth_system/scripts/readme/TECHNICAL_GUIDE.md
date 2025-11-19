# 深度技术指南 - 架构与实现细节

**目标受众**: 开发者、算法工程师 | **深度**: ⭐⭐⭐⭐⭐

---

## 目录
1. [架构总览](#架构总览)
2. [物理模型详解](#物理模型详解)
3. [代码详解](#代码详解)
4. [配置与集成](#配置与集成)
5. [扩展与优化](#扩展与优化)

---

## 架构总览

### 系统分层

```
┌─────────────────────────────────────────┐
│  ROS2 Nodes & Publishers                │
├─────────────────────────────────────────┤
│  CurrentAccelCalibNode                  │
│  ├─ 主状态机                            │
│  ├─ 轨迹跟踪 (Pure Pursuit)            │
│  └─ 电流控制 & 数据收集                 │
├─────────────────────────────────────────┤
│  Physics Models (新增)                   │
│  ├─ MotorModel                          │
│  ├─ CalibrationTier                     │
│  ├─ BrakingCalibrationMode              │
│  └─ TelemetryRecorder                   │
├─────────────────────────────────────────┤
│  ROS2 Topics                             │
│  ├─ /odom (EKF)                        │
│  ├─ /vesc/sensors (ERPM)                │
│  └─ /calib/ackermann_cmd                │
└─────────────────────────────────────────┘
```

### 数据流

```
EKF Odometry ──┐
               ├──→ PurePursuitController ──→ 目标电流 ──┐
VESC Sensors ──┘                                        │
                                                         ├──→ 发布 /calib/ackermann_cmd
状态机 ──→ CalibrationTier ──→ 目标速度 & 电流范围 ──┤
                                                         │
MotorModel ──→ 物理计算 (加速度, 拖曳力) ──────────────┤
                                                         │
TelemetryRecorder ←─── 记录所有参数 ←───────────────────┘
                            ↓
                      calibration_data.csv
```

---

## 物理模型详解

### 1. 简单线性模型

```
当前采用最小化假设：
  
  a = K * I / m
  
其中：
  a (m/s²): 加速度
  K (N/A): 电机常数 = 2.0 (启动值)
  I (A): 电流命令
  m (kg): 车辆质量 = 6.0

为什么这样设计？
  
  ✓ 简单：没有不确定的参数
  ✓ 实证：参数由实测数据确定，而不是假设
  ✓ 灵活：可根据实际结果改进
```

### 2. 为什么不用分段模型？

```
曾考虑的分段模型：
  低速: F_drag = 0.5 N
  中速: F_drag = 0.3 + 0.15*v
  高速: F_drag = 0.3 + 0.12*v + 0.05*v²

问题：
  ✗ 参数(0.5, 0.3, 0.15, 0.05)都是假设！
  ✗ 没有实测数据支撑
  ✗ 参数多 → 难以调试
  ✗ 可能与实际车型差异很大

改进方案：
  ✓ 先用简单模型收数据
  ✓ 分析数据中 I vs a 的关系
  ✓ 根据实际关系设计新模型
```

### 3. 数据驱动的改进流程

```
Step 1: 收集标定数据 (120秒)
  → 得到 6000+ 个 (I, a_measured) 点

Step 2: 分析数据特性
  → 画散点图
  → 看是否有速度相关性
  → 判断是否需要分段

Step 3: 拟合真实模型
  如果 I vs a 是线性的:
    a = k₁*I + b  (用最小二乘法)
  
  如果有速度依赖性:
    a = k₁*I + k₂*v + b  (线性回归)
  
  如果非线性:
    a = k₁*I + k₂*I² + b  (多项式拟合)

Step 4: 验证并集成
  → 用新参数更新 compute_acceleration()
  → 重新标定并验证
```

---

## 代码详解

### 1. MotorModel 类

```python
class MotorModel:
    """
    简化版电机模型：只做单位转换和加速度计算
    
    不包含拖曳力假设，保持最小化设计。
    """
    
    ERPM_TO_MS = 1.0 / 4650
    
    def __init__(self, vehicle_mass=6.0):
        self.vehicle_mass = vehicle_mass
    
    def erpm_to_speed(self, erpm):
        """ERPM → 线性速度 (m/s)"""
        return erpm * self.ERPM_TO_MS
    
    def compute_drag_force(self, v, mode='acceleration'):
        """
        Placeholder - returns 0 for now.
        
        After calibration data analysis, this will be updated
        with empirically-derived drag model if needed.
        """
        return 0.0
    
    def compute_acceleration(self, current_A, velocity_ms, mode='acceleration'):
        """
        Simple linear model: a = K * I / m
        
        K = 2.0 N/A (motor constant, refined by data)
        
        No drag force assumption here.
        
        Returns:
            a (float): 加速度 (m/s²)
        """
        K_motor = 2.0  # N/A
        F_motor = K_motor * current_A
        a = F_motor / self.vehicle_mass
        return a
```

**简化特点**:
- 无拖曳力假设
- K=2.0 N/A 作为启动值
- 参数由实测数据确定
- 保持代码简洁可读

---

### 2. CalibrationTier 类

```python
class CalibrationTier:
    """
    多层级标定管理器
    
    自动管理三个速度层级的电流递增
    """
    
    def __init__(self):
        self.tiers = [
            {'name': 'LOW_SPEED', 'target_v': 1.5, 'I_min': 5, 'I_max': 15, 'duration': 40},
            {'name': 'MID_SPEED', 'target_v': 3.0, 'I_min': 8, 'I_max': 20, 'duration': 40},
            {'name': 'HIGH_SPEED', 'target_v': 5.0, 'I_min': 10, 'I_max': 25, 'duration': 40},
        ]
        self.current_tier_idx = 0
        self.current_tier_start_time = None
    
    def get_current_stage(self, elapsed_time):
        """
        基于运行时间获取当前阶段信息
        
        Returns:
            dict: {
                'target_v': 目标速度,
                'current_A': 目标电流,
                'tier_name': 层级名称,
                'tier_idx': 层级索引 (0-2),
                'mode_flag': jerk值 (1=直线, 2=曲线)
            }
        """
        tier = self.tiers[self.current_tier_idx]
        tier_elapsed = elapsed_time % 40.0  # 每层40秒
        
        if self.current_tier_idx != elapsed_time // 40:
            self.current_tier_idx = int(elapsed_time // 40)
            if self.current_tier_idx >= 3:
                self.current_tier_idx = 2  # 限制在3层
        
        tier = self.tiers[min(self.current_tier_idx, 2)]
        progress = tier_elapsed / 40.0  # 0-1
        
        # 线性电流递增
        current_A = tier['I_min'] + progress * (tier['I_max'] - tier['I_min'])
        
        return {
            'target_v': tier['target_v'],
            'current_A': current_A,
            'tier_name': tier['name'],
            'tier_idx': self.current_tier_idx,
            'mode_flag': 1 if tier_elapsed < 20 else 2  # 前半段直线, 后半段曲线
        }
```

**功能**:
- 自动管理三层速度 (1.5/3.0/5.0 m/s)
- 每层线性电流递增
- 返回完整的阶段参数

---

### 3. BrakingCalibrationMode 类

```python
class BrakingCalibrationMode:
    """
    独立的制动标定模式
    
    使用负电流进行制动特性标定
    """
    
    def __init__(self):
        self.stages = [
            {'current_A': -5, 'target_decel': -0.5},    # Stage 1: -5A, 0-30s
            {'current_A': -10, 'target_decel': -1.0},   # Stage 2: -10A, 30-60s
            {'current_A': -15, 'target_decel': -1.5},   # Stage 3: -15A, 60-90s
            {'current_A': -20, 'target_decel': -2.0},   # Stage 4: -20A, 90-120s
        ]
    
    def get_braking_stage(self, elapsed_time):
        """
        基于运行时间获取当前制动阶段
        
        Returns:
            dict: {'current_A': 制动电流, 'target_decel': 目标减速度}
        """
        stage_idx = min(int(elapsed_time // 30), 3)  # 每阶段30秒
        return self.stages[stage_idx]
```

**特点**:
- 4阶段负电流递增 (-5A → -20A)
- 每阶段30秒
- 总时长120秒

---

### 4. TelemetryRecorder 类

```python
class TelemetryRecorder:
    """
    遥测数据记录器
    
    记录8个参数到CSV文件
    """
    
    def __init__(self, output_path='calibration_data.csv'):
        self.output_path = output_path
        self.data = {
            'timestamp': [],
            'current_A': [],
            'velocity_ms': [],
            'erpm': [],
            'steering_angle': [],
            'drag_force_N': [],
            'estimated_acceleration': [],
            'mode': []
        }
    
    def record(self, timestamp, current_A, velocity_ms, erpm, steering_angle, 
               drag_force_N, estimated_acceleration, mode):
        """记录单条数据行"""
        self.data['timestamp'].append(timestamp)
        self.data['current_A'].append(current_A)
        self.data['velocity_ms'].append(velocity_ms)
        self.data['erpm'].append(erpm)
        self.data['steering_angle'].append(steering_angle)
        self.data['drag_force_N'].append(drag_force_N)
        self.data['estimated_acceleration'].append(estimated_acceleration)
        self.data['mode'].append(mode)
    
    def save_to_csv(self):
        """导出所有数据到CSV"""
        import pandas as pd
        df = pd.DataFrame(self.data)
        df.to_csv(self.output_path, index=False)
        print(f"[TelemetryRecorder] 已保存 {len(df)} 行数据到 {self.output_path}")
```

**8列数据**:
1. timestamp - 相对时间 (秒)
2. current_A - 命令电流 (安培)
3. velocity_ms - 实测速度 (m/s)
4. erpm - 电机速度 (转/分钟)
5. steering_angle - 转向角 (弧度)
6. drag_force_N - 估算拖曳力 (牛顿)
7. estimated_acceleration - 预期加速度 (m/s²)
8. mode - 控制模式字符串

---

### 5. PurePursuitController 增强

```python
class PurePursuitController:
    """增强的Pure Pursuit轨迹追踪"""
    
    def compute_lookahead_distance(self, velocity, gain=1.5):
        """
        自适应前视距离
        
        前视距离 = gain × velocity
        
        低速: 小前视距离 (精确跟踪)
        高速: 大前视距离 (平稳轨迹)
        """
        return max(0.3, gain * velocity)
    
    def is_in_curve(self, steering_angle, threshold=0.1):
        """
        检测是否在曲线上
        
        |δ| > threshold → 曲线
        |δ| ≤ threshold → 直线
        """
        return abs(steering_angle) > threshold
```

**改进**:
- 动态前视距离 (速度自适应)
- 曲线检测 (基于转向角)
- 用于模式切换

---

### 6. 主回调函数 - control_loop_callback

```python
def control_loop_callback(self):
    """
    主控制循环 (50 Hz)
    
    流程:
    1. 获取当前状态 (位置, 速度, 方向)
    2. 获取目标参数 (速度, 电流)
    3. 判断控制模式 (曲线 vs 直线)
    4. 输出命令 (转向角, 电流/加速度)
    5. 记录遥测数据
    """
    
    # 1. 获取EKF位置和速度
    x, y = self.current_pose
    vx, vy = self.current_twist
    v_linear = sqrt(vx**2 + vy**2)
    
    # 2. 获取目标参数
    stage_info = self.calib_tier.get_current_stage(self.elapsed_time)
    target_v = stage_info['target_v']
    target_I = stage_info['current_A']
    
    # 3. 计算轨迹跟踪 (Pure Pursuit)
    steering_angle = self.pp_controller.compute_steering_angle(
        self.current_pose, self.trajectory
    )
    
    # 4. 判断模式
    if self.pp_controller.is_in_curve(steering_angle):
        # 曲线: 速度控制模式
        mode_flag = 1  # jerk = 1.0
        command_value = target_v  # 速度命令
    else:
        # 直线: 电流控制模式
        mode_flag = 2  # jerk = 2.0
        command_value = target_I  # 电流命令
    
    # 5. 发布命令
    msg = AckermannDriveStamped()
    msg.drive.steering_angle = steering_angle
    msg.drive.acceleration = command_value
    msg.drive.jerk = float(mode_flag)
    self.cmd_pub.publish(msg)
    
    # 6. 记录遥测
    drag_force = self.motor_model.compute_drag_force(v_linear)
    est_acc = self.motor_model.compute_acceleration(target_I, v_linear)
    
    self.telemetry.record(
        timestamp=self.elapsed_time,
        current_A=target_I,
        velocity_ms=v_linear,
        erpm=int(v_linear / self.motor_model.ERPM_TO_MS),
        steering_angle=steering_angle,
        drag_force_N=drag_force,
        estimated_acceleration=est_acc,
        mode='CURVE' if mode_flag == 1 else 'LINE'
    )
```

---

## 配置与集成

### 1. Launch文件配置

```python
# calib_launch.py

def generate_launch_description():
    
    calibration_mode = LaunchConfiguration('calibration_mode')
    wheelbase = LaunchConfiguration('wheelbase')
    lookahead_gain = LaunchConfiguration('lookahead_gain')
    figure8_radius = LaunchConfiguration('figure8_radius')
    command_frequency = LaunchConfiguration('command_frequency')
    vehicle_mass = LaunchConfiguration('vehicle_mass')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('calibration_mode', default_value='acceleration',
                             description='acceleration or braking'),
        DeclareLaunchArgument('vehicle_mass', default_value='6.0'),
        DeclareLaunchArgument('figure8_radius', default_value='1.6'),
        
        # 节点启动
        Node(
            package='f1tenth_system',
            executable='current_acc_calib_node',
            output='screen',
            parameters=[
                {'calibration_mode': calibration_mode},
                {'vehicle_mass': vehicle_mass},
                {'figure8_radius': figure8_radius},
                # ... 其他参数
            ],
            remappings=[
                ('/calib/ackermann_cmd', '/cmd_vel'),  # 重新映射输出
                ('/vesc/sensors', '/vesc_feedback'),   # VESC订阅
            ]
        )
    ])
```

### 2. ROS2消息映射

```yaml
# 发布 → Ackermann命令
Topic: /calib/ackermann_cmd
Type: ackermann_msgs/AckermannDriveStamped

# 可选重新映射
remappings:
  - ['/calib/ackermann_cmd', '/cmd_vel']  # 发送给Ackermann Mux
  - ['/odom', '/odometry/filtered']       # 从EKF接收
  - ['/vesc/sensors', '/motor_feedback']  # VESC数据
```

### 3. 参数文件

```yaml
# params/calibration.yaml

calibration:
  mode: 'acceleration'  # or 'braking'
  duration_seconds: 120
  
trajectory:
  figure8_radius: 1.6
  wheelbase: 0.33
  
control:
  command_frequency: 50  # Hz
  lookahead_gain: 1.5
  
physics:
  vehicle_mass: 6.0
  motor_constant: 2.0  # N/A
  erpm_to_ms: 0.000215  # 1/4650
  
drag_model:
  coulomb_friction: 0.5      # 低速
  linear_drag_a: 0.3         # 中速常数
  linear_drag_b: 0.15        # 中速系数
  quadratic_drag: 0.05       # 高速系数
```

---

## 扩展与优化

### 0. 数据驱动的模型精化 ⭐ 首先做这个

标定后，用 Python 从 rosbag 提取数据并分析：

```python
from rosbag2_py import SequentialReader
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# 1. 从 bag 提取数据
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
        data.append({'t': timestamp/1e9, 'v': v})
    elif topic == '/calib/ackermann_cmd' and data:
        data[-1]['I'] = msg.drive.acceleration
        data[-1]['delta'] = msg.drive.steering_angle
    elif topic == '/vesc/sensors' and data:
        data[-1]['erpm'] = msg.state.electrical_rpm

df = pd.DataFrame(data)

# 2. 计算加速度（从速度变化）
df['a'] = df['v'].diff() / df['t'].diff()

# 3. 分析 I vs a 关系
print("=== 线性拟合: a = k*I + b ===")

def linear(I, k, b):
    return k * I + b

# 删除 NaN 值
valid = df[['I', 'a']].dropna()

params, _ = curve_fit(linear, valid['I'], valid['a'])
print(f"a = {params[0]:.4f}*I + {params[1]:.4f}")

# 4. 按速度分层分析
print("\n=== 按速度分层 ===")
for v_min, v_max, name in [(0, 1.0, '低速'), (1.0, 3.0, '中速'), (3.0, 10, '高速')]:
    subset = valid[(valid['v'] >= v_min) & (valid['v'] < v_max)]
    if len(subset) > 10:
        p, _ = curve_fit(linear, subset['I'], subset['a'])
        print(f"{name}: a = {p[0]:.4f}*I + {p[1]:.4f}")

# 5. 可视化
plt.figure(figsize=(12, 4))
plt.scatter(valid['I'], valid['a'], alpha=0.3)
plt.plot(valid['I'].sort_values(), linear(valid['I'].sort_values(), *params), 'r-')
plt.xlabel('Current (A)')
plt.ylabel('Acceleration (m/s²)')
plt.title('I vs a')
plt.grid(True)
plt.show()
```



```python
def compute_drag_force_with_temp(v, temp_celsius):
    """
    温度相关的拖曳力计算
    
    温度升高 → 轮胎变软 → 摩擦减少
    """
    base_drag = compute_drag_force(v)
    
    # 温度系数 (-0.002 per °C)
    temp_offset = (temp_celsius - 20.0) * (-0.002)
    
    return base_drag * (1.0 + temp_offset)
```

### 2. 添加路面检测

```python
def detect_surface_type(imu_data):
    """
    基于IMU加速度的路面检测
    """
    accel_magnitude = sqrt(ax**2 + ay**2 + az**2)
    
    if accel_magnitude < 10.2:
        return 'SMOOTH'      # 光滑路面
    elif accel_magnitude < 10.5:
        return 'NORMAL'      # 普通路面
    else:
        return 'ROUGH'       # 粗糙路面
```

### 3. 添加轮子打滑检测

```python
def detect_wheel_slip(velocity_odom, velocity_erpm):
    """
    比较里程计速度和电机速度，检测打滑
    """
    slip_ratio = abs(velocity_odom - velocity_erpm) / (velocity_erpm + 1e-6)
    
    if slip_ratio > 0.1:
        return True  # 打滑 > 10%
    else:
        return False
```

### 4. 添加自适应电流限制

```python
def compute_adaptive_current_limit(velocity, steering_angle):
    """
    基于速度和转向角的动态电流限制
    """
    base_limit = 25.0  # 直线最大电流
    
    # 转向时降低电流限制 (防止打滑)
    steering_factor = 1.0 - abs(steering_angle) / 0.5
    
    # 高速时降低电流限制 (防止失控)
    speed_factor = 1.0 - (velocity - 5.0) / 5.0 if velocity > 5.0 else 1.0
    
    adaptive_limit = base_limit * steering_factor * speed_factor
    
    return max(5.0, adaptive_limit)
```

---

## 性能优化

### 1. 向量化计算

```python
# 替代循环的NumPy计算
import numpy as np

# 批量转换 ERPM → 速度
erpm_array = np.array([4650, 9300, 14000])
velocity_array = erpm_array / 4650  # 向量化
```

### 2. 缓存频繁计算

```python
class CachedMotorModel(MotorModel):
    def __init__(self):
        super().__init__()
        self._drag_cache = {}
    
    def compute_drag_force(self, v, mode='acceleration'):
        key = (round(v, 2), mode)
        if key not in self._drag_cache:
            self._drag_cache[key] = super().compute_drag_force(v, mode)
        return self._drag_cache[key]
```

### 3. 多线程数据处理

```python
import threading

class AsyncTelemetryRecorder(TelemetryRecorder):
    def __init__(self):
        super().__init__()
        self.write_thread = threading.Thread(target=self._write_worker, daemon=True)
        self.write_thread.start()
    
    def _write_worker(self):
        # 后台线程定期刷新数据
        while True:
            if len(self.data['timestamp']) > 1000:
                self._flush_to_disk()
            time.sleep(10)  # 每10秒检查一次
```

---

## 测试与验证

### 单元测试

```python
# test_calib_components.py

def test_motor_model():
    """测试电机模型"""
    motor = MotorModel(vehicle_mass=6.0)
    
    # 测试ERPM转换
    assert motor.erpm_to_speed(4650) == 1.0
    assert motor.erpm_to_speed(9300) == 2.0
    
    # 测试拖曳力分段
    assert motor.compute_drag_force(0.5) == 0.5  # 低速
    assert 0.3 < motor.compute_drag_force(2.0) < 0.6  # 中速
    assert motor.compute_drag_force(4.0) > 1.0  # 高速
    
    # 测试加速度
    a = motor.compute_acceleration(10, 1.0)
    assert a > 0  # 应该有正加速度

def test_calibration_tier():
    """测试标定层级"""
    tier = CalibrationTier()
    
    # 测试第一层
    stage = tier.get_current_stage(0)
    assert stage['target_v'] == 1.5
    assert stage['tier_name'] == 'LOW_SPEED'
    
    # 测试电流递增
    stage1 = tier.get_current_stage(10)
    stage2 = tier.get_current_stage(20)
    assert stage2['current_A'] > stage1['current_A']
    
    # 测试层级切换
    stage = tier.get_current_stage(45)
    assert stage['target_v'] == 3.0
    assert stage['tier_name'] == 'MID_SPEED'

def test_telemetry_recorder():
    """测试遥测记录"""
    rec = TelemetryRecorder('/tmp/test.csv')
    
    for i in range(100):
        rec.record(i*0.02, 10+i*0.05, 1.0+i*0.01, 4650+i, 0.05, 0.5, 3.0, 'LINE')
    
    rec.save_to_csv()
    
    # 验证输出
    import pandas as pd
    df = pd.read_csv('/tmp/test.csv')
    assert len(df) == 100
    assert list(df.columns) == [...]
```

---

## 故障排除 - 技术版

### 问题1: 加速度数据异常高/低

```
症状: estimated_acceleration > 10 m/s² 或 < -5 m/s²

原因排查:
1. 检查电流命令是否正确
   ros2 topic echo /calib/ackermann_cmd | grep acceleration
   
2. 检查速度反馈是否准确
   ros2 topic echo /odom | grep twist
   
3. 检查电机常数 K_MOTOR 是否正确
   计算: F_motor = K_MOTOR * I_max
   应该 ≈ 50 N (对于 25A 电流)
   
解决方案:
   调整 compute_acceleration 中的 K_MOTOR 参数
   或增大 vehicle_mass 参数
```

### 问题2: CSV数据不完整

```
症状: calibration_data.csv 行数 < 6000

原因排查:
1. 运行时间不足120秒
   检查: tail calibration_data.csv | wc -l
   
2. 数据采样漏失
   原因: 控制频率下降 (CPU过载)
   解决: 降低 command_frequency 参数
   
3. 磁盘满
   检查: df -h
   清理磁盘空间
```

### 问题3: 轨迹跟踪不稳定

```
症状: steering_angle 震荡, 车偏离8字形轨迹

原因排查:
1. lookahead_gain 过小 → 过度反应
   解决: 增加 lookahead_gain (例如 1.5 → 2.0)
   
2. EKF噪声过大
   检查: ros2 topic hz /odom
   应该 > 20 Hz
   
3. 轮子打滑
   检查: velocity_ms vs ERPM/4650 的差异
   解决: 增大 figure8_radius 或降低目标速度
```

---

**深度技术指南完** | 适合架构设计和调试工作
