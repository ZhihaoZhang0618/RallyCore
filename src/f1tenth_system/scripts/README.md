# 纵向标定采集（有定位 / 无定位）

本目录下目前有两套纵向标定采集方案：

- **有定位方法（Pure Pursuit 自动跑圈）**：依赖里程计/定位（Odometry），车辆自动沿“跑道/8字”轨迹行驶；直线自动扫电流、弯道自动定速，更适合长时间、可重复的采集。
- **无定位方法（Manual Steer 遥控介入）**：不依赖定位；纵向由脚本按“直线扫电流 / 弯道定速”发布，横向由遥控器介入，更适合你当前遇到 fastlio/IMU 漂移或定位不稳定时的采集。

两套方案都统一输出话题：`/calib/ackermann_cmd`（中控/下游需按本文的 jerk 约定解析）。

---

## 0.1) 构建与环境准备（必须）

本目录脚本通过 `ament_python` 安装为可执行文件，首次使用或脚本更新后请先编译：

```bash
cd ~/RallyCore
colcon build --packages-select f1tenth_system
```

然后在**当前终端** `source`（按你当前 shell 选择对应脚本）：

```bash
# bash
source install/setup.bash

# zsh
source install/setup.zsh
```

---

## 0.2) `/calib/ackermann_cmd` 消息约定（中控必须按此解析）

本仓库用 `AckermannDriveStamped.drive.jerk` 作为“模式标志位”，用于在同一个 topic 上区分 speed/current：

- `AckermannDriveStamped.drive.jerk == 2.0`：电流模式（current mode）
   - `drive.acceleration` = 电流(A)
   - `drive.steering_angle` = 转向(rad)
   - `drive.speed` 不使用（可置 0）
- `AckermannDriveStamped.drive.jerk == 0.0`：速度模式（speed mode）
   - `drive.speed` = 目标速度(m/s)
   - `drive.steering_angle` = 转向(rad)
   - `drive.acceleration` 可置 0

---

## 1) 有定位方法：Pure Pursuit 自动跑圈标定（依赖 /odom）

### 1.1 适用场景

- 能稳定提供 `nav_msgs/Odometry`（例如 `/odom` 来自 EKF / fastlio / 轮速里程计融合等）。
- 希望“可重复、可长期跑”的标定采集：直线段自动扫电流、弯道段自动定速返回目标速度区间。

### 1.2 核心脚本

#### (1) `pp_current_acc_calib.py`：自动跑圈 + 电流/速度分段标定

脚本：`src/f1tenth_system/scripts/pp_current_acc_calib.py`

订阅：
- `/odom`（`nav_msgs/Odometry`）
- `/vesc/sensors`（`vesc_msgs/VescStateStamped`，用于 ERPM/遥测）

发布：
- `/calib/ackermann_cmd`（`ackermann_msgs/AckermannDriveStamped`）
- `/calib/lookahead_point`（`geometry_msgs/PointStamped`，用于可视化）

运行：

```bash
ros2 run f1tenth_system pp_current_acc_calib.py
```

常用参数（示例）：

```bash
# 标定模式：acceleration（正向加速扫电流）/ braking（负电流制动标定）
ros2 param set /current_acc_calib calibration_mode acceleration

# acceleration 模式下的电流斜率（A/s）
ros2 param set /current_acc_calib current_ramp_rate 5.0

# 跑道尺寸（建议先从更大半径/更长直道开始，降低横向扰动）
ros2 param set /current_acc_calib track_radius 3.0
ros2 param set /current_acc_calib track_straight_length 10.0

# PP 参数（可先用 pp_param_tuner 调好，再搬过来）
ros2 param set /current_acc_calib lookahead_gain 1.6
ros2 param set /current_acc_calib min_lookahead 0.3
ros2 param set /current_acc_calib max_lookahead 3.5
ros2 param set /current_acc_calib lateral_error_gain 1.0
ros2 param set /current_acc_calib heading_error_gain 0.4
ros2 param set /current_acc_calib curvature_ff_gain 0.1

# 轨迹相对定位坐标系的偏置（现场对齐用，可在线调）
ros2 param set /current_acc_calib traj_offset_x 0.0
ros2 param set /current_acc_calib traj_offset_y 0.0
ros2 param set /current_acc_calib traj_offset_yaw 0.0
```

运行逻辑（简述）：
- 轨迹为“跑道/8字”闭环：直线段用于标定、弯道段用于稳定车速。
- **直线段**：发布 `jerk=2.0`（current mode），`drive.acceleration` 填“电流(A)”并按 `current_ramp_rate` 连续上升。
- **弯道段**：发布 `jerk=0.0`（speed mode），按当前 tier 的目标速度（例如 1.5/3.0/5.0m/s）定速运行。

结束行为：达到当前模式定义的终止条件后，会发布停止命令（speed=0 / current=0）并日志提示完成。

rosbag 录制建议（用于后处理建模/诊断）：

```bash
ros2 bag record -o pp_calib \
   /odom \
   /vesc/sensors \
   /calib/ackermann_cmd \
   /calib/lookahead_point \
   /tf /tf_static
```

#### (2) `pp_param_tuner.py`：仅用于 Pure Pursuit 参数调参（非必需，但强烈建议先跑）

脚本：`src/f1tenth_system/scripts/pp_param_tuner.py`

用途：把 PP 的 lookahead / gain / 高速转向衰减 / 轨迹尺寸等调到“不会扭来扭去、不会冲出弯”的状态，再去跑 `pp_current_acc_calib.py` 的标定采集。

注意：该节点默认接口与标定节点不同：
- 订阅：`/odometry/filtered`
- 发布：`/drive`

如果你的系统实际使用的是 `/odom` 或需要输出到 `/ackermann_cmd`，建议用 remap/bridge 适配（不改代码）。

快速运行：

```bash
ros2 run f1tenth_system pp_param_tuner.py --ros-args -p target_speed:=2.0
```

推荐流程（建议先稳住横向，再做纵向标定）：

1) 先跑调参：`pp_param_tuner`（低速开始，把 cte/摆动压下去）
2) 再跑采集：`pp_current_acc_calib`（扩大半径/直道，降低横向扰动）

接口适配（常见 remap 示例，不改代码）：

```bash
# 1) 如果你的定位里程计话题是 /odom（pp_param_tuner 默认订阅 /odometry/filtered）
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /odometry/filtered:=/odom

# 2) 如果你的车辆执行入口是 /ackermann_cmd（pp_param_tuner 默认发布 /drive）
ros2 run f1tenth_system pp_param_tuner.py --ros-args \
   -r /drive:=/ackermann_cmd
```

---

## 2) 无定位方法：Manual Steer 标定采集（无定位依赖）

目的：绕开 fastlio/IMU 漂移问题，让采集过程不依赖定位。

核心逻辑（与中控配合）：
- **直线**（遥控器方向在死区内）：发布 **电流模式**，电流从 0→60A 连续扫掠（用于标定）
- **弯道**（遥控器方向超死区）：发布 **速度模式**，按阶段提前设定的速度行驶（用于稳定车速与回到目标速度区间）
- **横向（steering）**：由遥控器介入；直线时强制 steering=0，弯道时 steering=遥控器

输出：`/calib/ackermann_cmd`

> 注意：严格电流扫掠版会在直线发布 jerk=2.0（current mode），弯道发布 jerk=0.0（speed mode）。

---

## 3) 新标定流程（推荐）：先定速测电流，再分速度区间测加速度

你反馈原方法效果不佳时，建议改为“两阶段”采集：

### 3.1 阶段 A：定速 1..8 m/s，测保持速度的平均相电流

目标：尽量在 **直线（steering=0）** 的情况下，分别测试 1、2、...、8 m/s 的保持电流；每个速度保持 `hold_time_sec`（默认 10s，可人工设置）。

建议：阶段 A 为了减少横向扰动，推荐关闭 RC 介入（`use_rc_steering:=false`），或确保遥控器方向始终在死区内。

> 如果开启了 `use_rc_steering:=true`：脚本会在“转弯（出死区）”期间不采样；并在“转弯结束回到直线（进死区）”后额外等待 `post_turn_settle_sec`，再开始采样，避免弯道残余电流抬升污染结果。

脚本：`src/f1tenth_system/scripts/speed_hold_current_logger.py`

运行示例：

```bash
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
   -p speeds:="[1,2,3,4,5,6,7,8]" \
   -p hold_time_sec:=10.0 \
   -p use_rc_steering:=false \
   -p vesc_topic:=/sensors/core \
   -p output_path:=speed_hold_current_results.txt
```

（如确实需要 RC 介入运行）

```bash
ros2 run f1tenth_system speed_hold_current_logger.py --ros-args \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

输出：
- `speed_hold_current_results.txt`：每个速度对应的平均相电流（默认策略：先等待车速达到目标并稳定一段时间，再开始采样）
- 可选 `csv_path`：记录全量时间序列，便于排查

### 3.2 阶段 B：对每个速度区间 v→v+1，做电流阶梯扫描测加速度

目标：对 1→2、2→3、...、7→8 m/s 区间，基于阶段 A 的“基准电流”起步，以 `current_step`（默认 3A）逐步增大到 `current_max`（默认 80A），测到达时间 $t$，估算加速度 $a = \Delta v / t$。

脚本：`src/f1tenth_system/scripts/speed_interval_accel_sweep.py`

运行示例：

```bash
ros2 run f1tenth_system speed_interval_accel_sweep.py --ros-args \
   -p v_start:=1.0 -p v_end:=8.0 -p dv:=1.0 \
   -p base_current_file:=speed_hold_current_results.txt \
   -p current_step:=3.0 -p current_max:=80.0 \
   -p vesc_topic:=/sensors/core \
   -p odom_topic:=/odom \
   -p output_path:=speed_interval_accel_results.txt
```

（如确实需要 RC 介入运行，且只在直线段做 trial）

```bash
ros2 run f1tenth_system speed_interval_accel_sweep.py --ros-args \
   -p use_rc_steering:=true \
   -p rc_topic:=/rc/channels \
   -p rc_timeout_sec:=0.25 \
   -p post_turn_settle_sec:=0.8
```

输出：
- `speed_interval_accel_results.txt`：每条记录为 `v0 v1 current_A t_sec accel reached`

备注：该脚本会在每次试验前先用 speed mode 把车“拉回/稳定到 v0”，再切到 current mode 计时；这样能尽量隔离“速度稳定后→升档加速段”的电流影响。

> 速度反馈来自 `odom.twist.twist.linear.x`（由 `odom_topic` 指定），电流反馈来自 `vesc_topic`。

---

### 3.3 离线分析 Prompt（阶段 A+B 输出文件）

把下面 prompt 里的路径替换成你实际生成的文件路径，直接丢给分析助手即可。

```text
你是车辆纵向动力学/标定工程师。请基于两阶段采集的输出文件，建立“速度分段的电流→加速度映射”，并给出可落地的查表/拟合结果与质量诊断。

输入文件（把路径替换成实际文件）：
1) 阶段A：speed_hold_current_results.txt
   - 格式：speed_mps  mean_current_A  std_current_A  samples
2) 阶段B：speed_interval_accel_results.txt
   - 格式：v0  v1  current_A  t_sec  accel_mps2  reached
可选：
- 阶段A/阶段B 的 csv_path（若存在），用于排查异常与可视化

采集约定/重要前提：
- 弯道期间不采样/不做 trial；回正后会等待 post_turn_settle_sec 再开始采样/试验。
- 阶段B：每次 trial 前会先用 speed mode 拉回并稳定到 v0，再切 current mode 计时加速到 v1。
- 速度反馈来自 /odom.twist.twist.linear.x（已体现在数据里），电流反馈来自 VESC 遥测（相电流/iq 等字段）。

你需要完成的工作：
1) 读取与检查
   - 解析两个 txt，列出阶段A速度点的数量、速度范围、是否有缺失/NaN
   - 解析阶段B所有 trial，统计 reached=1 的比例；按区间(v0->v1)分组统计样本数
   - 找出明显异常：t_sec<=0、accel_mps2 非法、current_A 乱序或重复等

2) 阶段A：基线电流曲线
   - 画出 I_base(v)=mean_current_A vs v
   - 对 I_base 做平滑/插值（例如线性插值或分段拟合），输出一个可查询的函数/表
   - 给出每个速度点的 std 与样本数，提示置信度

3) 阶段B：建立“电流→加速度”模型（按速度区间分段）
   对每个速度区间（例如 1->2, 2->3, ...）：
   - 只使用 reached=1 的 trial 进行拟合
   - 画出散点：current_A vs accel_mps2
   - 提供两种建模结果并对比：
     A) 直接拟合：a = k*I + b（线性回归）
     B) 扣基线电流：I_eff = I - I_base(v0)，拟合 a = k*I_eff + b 或 a=k*I_eff
   - 给出每段的 k,b，R²、RMSE，并指出是否存在非线性/饱和（高电流段加速度提升变慢）

4) 输出可落地结果（非常重要）
   - 生成“查表形式”的结果：对每个速度区间输出一张 I->a 的单调表（必要时做单调化）
   - 或输出分段线性参数表：每段 (v0->v1) 的 k,b，以及推荐使用范围
   - 给出反算形式建议：给定目标加速度 a* 时，如何得到电流指令 I*
   - 建议加入约束：I_max、最小可用电流、超时/未到达的处理

5) 质量诊断与下一轮建议
   - 哪些区间数据太少/噪声大（建议增加 repeats 或延长 reset/stable 时间）
   - 哪些电流档经常 timeout（提示 current_max、trial_timeout_sec 或 dv 需要调整）
   - 建议的 post_turn_settle_sec、speed_tolerance、stable_required_sec 的经验取值范围

输出要求：
- 给出关键图表列表（你会画哪些图、每图说明）
- 给出最终建议的表格/参数（清晰列出每个区间的 k,b 或查表点）
- 如果需要插值，说明插值方法与边界处理方式（超出速度范围怎么办）
```

---

> `/calib/ackermann_cmd` 的 jerk 模式约定见上方「0.2) 消息约定」。

---

## 重要提示（请先读）

- 本文当前**主要维护/推荐**的是上面的「3) 新标定流程（两阶段：A 定速测电流 + B 分速度区间测加速度）」。
- 从这里往下的内容以“补充资料/历史脚本说明”为主，可能不会与代码保持完全同步；如有不一致，**以脚本代码与上方第 3 节为准**。

---

### 1) 新增节点

### 1.1 严格电流扫掠版（推荐用于标定）

脚本：`src/f1tenth_system/scripts/manual_steer_current_sweep_stages.py`

运行：

```bash
ros2 run f1tenth_system manual_steer_current_sweep_stages.py
```

常用参数：

```bash
# 三阶段速度（m/s）
ros2 param set /manual_steer_current_sweep_stages stage_speeds "[1.5, 3.0, 5.0]"

# 电流扫掠：0..60A，ramp 5A/s
ros2 param set /manual_steer_current_sweep_stages current_min 0.0
ros2 param set /manual_steer_current_sweep_stages current_max 60.0
ros2 param set /manual_steer_current_sweep_stages current_ramp_rate 5.0

# 直线/弯道判定与转向映射（与 joystick_control_v2 一致的参数名）
ros2 param set /manual_steer_current_sweep_stages steering_channel 4
ros2 param set /manual_steer_current_sweep_stages steering_channel_mid 984
ros2 param set /manual_steer_current_sweep_stages channel_deadzone 100
ros2 param set /manual_steer_current_sweep_stages steering_limit 0.40
ros2 param set /manual_steer_current_sweep_stages steering_reverse true
```

结束行为：三阶段（每阶段电流扫到 60A）完成后会发布 `speed=0` 与 `current=0` 并打印 `[DONE]`。

### 1.2 仅速度版（可用于对照/排查链路）

脚本：`src/f1tenth_system/scripts/manual_steer_speed_stages.py`

运行：

```bash
ros2 run f1tenth_system manual_steer_speed_stages.py
```

常用参数：

```bash
# 三阶段速度（m/s）与时长（s）
ros2 param set /manual_steer_speed_stages stage_speeds "[1.5, 3.0, 5.0]"
ros2 param set /manual_steer_speed_stages stage_durations "[60.0, 60.0, 60.0]"

# RC话题、输出话题
ros2 param set /manual_steer_speed_stages rc_topic /rc/channels
ros2 param set /manual_steer_speed_stages cmd_topic /calib/ackermann_cmd

# 转向判0（死区）与转向映射（与 joystick_control_v2 一致的参数名）
ros2 param set /manual_steer_speed_stages steering_channel 4
ros2 param set /manual_steer_speed_stages steering_channel_mid 984
ros2 param set /manual_steer_speed_stages channel_deadzone 100
ros2 param set /manual_steer_speed_stages steering_limit 0.40
ros2 param set /manual_steer_speed_stages steering_reverse true
```

结束行为：三阶段结束后会发布 `speed=0, steering=0` 并打印 `[DONE]`。

---

### 2) rosbag 录制

你当前的命令：

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

---

### 3) 分析 rosbag 的 Prompt（可直接丢给分析助手）

把 `{bag_dir}` 替换成你的 bag 路径。

```text
你是车辆纵向动力学/标定工程师。请基于一次“手动转向介入 + 三阶段定速”的 rosbag 数据，建立速度分阶段/分桶的纵向模型，并评估电流与加速度的关系。

背景与约定：
- 上游节点发布 /calib/ackermann_cmd（AckermannDriveStamped）。本方案 drive.jerk=0.0（speed mode）。
- 当遥控器方向在死区内：认为直线（steering=0）。
- 当遥控器方向超出死区：认为弯道（steering=遥控器值）。
- 电机遥测来自 /sensors/core（VescStateStamped），包含 speed、avg_iq（电流相关）等。

数据位置：
- rosbag 目录：{bag_dir}

请你完成：
1) 数据检查：列出 bag 中所有 topic、消息数、估计频率；检查时间戳是否单调、是否有长间隙。
2) 对齐与派生量：
   - 用 /sensors/core/state/speed 和 /odom.twist.twist.linear.x 互相交叉验证车速（说明可能的比例/延迟差）。
   - 从 /odom 速度计算加速度 a（说明差分、低通滤波/平滑、延迟补偿策略）。
3) 直线/弯道分段：
   - 首选用 /rc/channels 的转向通道按“与 joystick_control_v2 一致的死区规则”判定直线段；
   - 备选用 /calib/ackermann_cmd.drive.steering_angle 绝对值阈值判定。
4) 分阶段统计：
   - 按三阶段目标速度区间分段（或按速度区间自动聚类）。
   - 每阶段输出：平均速度、速度方差、平均电流(或 avg_iq)、平均加速度、加速度噪声水平。
5) 建模与结论：
   - 给出至少一个可落地的模型：例如 a = k(v)*I + b(v)（k、b 随速度分段常数），或直接用查表策略。
   - 报告 R² / RMSE，并指出主要误差来源（弯道横向扰动、地面坡度、电池电压变化、延迟）。
6) 质量诊断：
   - 是否存在“直线段仍有转向输入/车辆摇摆”的污染；
   - 是否存在电流饱和、死区或速度闭环震荡。
7) 下一轮采集建议：
   - 每阶段应该维持的最小直线时间、建议的目标速度范围；
   - 需要补录的 topic（若缺失就说明影响）。
8）考虑占空比达到最大的情况，并给出这种情况下的标定代码建议

输出要求：
- 给出最终建议的映射形式（公式/分段参数/查表），以及你使用的筛选规则（阈值、时间窗口、滤波参数）。
```

---

### 4) 备注：关于“/calib/ackermann_cmd”如何接入

本节点**只负责上游发布** `/calib/ackermann_cmd`。
如果你当前的控制链路不是直接使用该 topic，请用 launch remap 在不修改任何上下游代码的前提下接入。

示例（把输出 remap 到真正生效的 ackermann 入口）：

```bash
ros2 run f1tenth_system manual_steer_speed_stages.py \
  --ros-args -r /calib/ackermann_cmd:=/ackermann_cmd
```
