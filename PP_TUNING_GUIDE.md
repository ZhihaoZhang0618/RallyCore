# Pure Pursuit 参数调优指南

## 📊 当前Bag分析结果

根据你录制的bag文件分析：

### 性能指标
- **直线路段RMS误差**: 64.9cm ⚠️ **偏大**
- **弯道路段RMS误差**: 120.6cm 
- **平均速度**: 3.05 m/s
- **最大速度**: 5.15 m/s
- **转向抖动**: 平均 0.37 rad/s（良好）

### 主要问题
1. ⚠️ **直线路段跟踪精度不足** (64.9cm RMS，目标应<10cm)
2. ⚠️ **高速时发生侧滑** (>5m/s时轮胎摩擦声剧烈)

---

## 🎯 参数更新总结

### 已按图片设置的参数

| 参数 | 旧值 | 新值 | 说明 |
|------|------|------|------|
| `lookahead_gain` | 2.2 | **0.495** | Lookahead距离增益（较小值更激进） |
| `min_lookahead` | 0.50 | **0.30** | 最小前视距离（米） |
| `max_lookahead` | 3.5 | **3.5** | 最大前视距离（米） |
| `lateral_error_gain` | 0.7 | **1.0** | 横向误差增益 |
| `heading_error_gain` | 0.2 | **0.2** | 航向误差增益 |
| `curvature_ff_gain` | 0.08 | **0.08** | 曲率前馈增益 |
| `track_radius` | 1.5 | **3.0** | 赛道转弯半径（米） |
| `track_straight_length` | 3.0 | **20.0** | 直道长度（米） |
| `target_speed` | 1.0 | **5.5** | 目标速度（m/s） |

### ⭐ 新增：高速转向衰减参数

为解决>5m/s时的侧滑问题，新增以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_scale_speed` | **7.0** | 开始转向衰减的速度（m/s） |
| `end_scale_speed` | **8.0** | 达到最大衰减的速度（m/s） |
| `steer_downscale_factor` | **0.80** | 最大衰减系数（80% = 20%衰减） |

**工作原理**（类似你的第二张图）：
```
速度 < 7.0 m/s:  转向角 = 100% (无衰减)
速度 = 7.0 m/s:  转向角 = 100% (开始衰减)
速度 = 7.5 m/s:  转向角 = 90%  (线性插值)
速度 ≥ 8.0 m/s:  转向角 = 80%  (最大衰减)
```

---

## 🔧 针对直线跟踪的建议调整

基于分析，直线RMS误差64.9cm太大。建议逐步尝试：

### 方案1：增加横向误差增益（最有效）
```bash
ros2 param set /pp_param_tuner lateral_error_gain 1.2
# 观察效果，如果还不够贴线，继续增加
ros2 param set /pp_param_tuner lateral_error_gain 1.4
```

### 方案2：增加航向误差增益
```bash
ros2 param set /pp_param_tuner heading_error_gain 0.3
# 或更激进
ros2 param set /pp_param_tuner heading_error_gain 0.4
```

### 方案3：减小最小前视距离（谨慎）
```bash
ros2 param set /pp_param_tuner min_lookahead 0.25
# 或更小（但可能导致抖动）
ros2 param set /pp_param_tuner min_lookahead 0.20
```

### 方案4：组合调整（推荐）
```bash
# 同时调整多个参数
ros2 param set /pp_param_tuner lateral_error_gain 1.3
ros2 param set /pp_param_tuner heading_error_gain 0.3
ros2 param set /pp_param_tuner min_lookahead 0.25
```

---

## 🧪 高速测试流程

1. **启动节点**：
```bash
ros2 run f1tenth_system pp_param_tuner
```

2. **逐步提高速度测试**：
```bash
# 从低速开始
ros2 param set /pp_param_tuner target_speed 3.0
# 观察稳定后，逐步提高
ros2 param set /pp_param_tuner target_speed 5.0
ros2 param set /pp_param_tuner target_speed 6.0
ros2 param set /pp_param_tuner target_speed 7.0
```

3. **观察高速转向衰减**：
当速度>7m/s时，日志会显示：
```
v=7.50m/s -> target=8.00 | δ=0.245rad scale=0.90 | ld=4.20m | cte=0.050m
                                      ^^^^^^^^^^^
                                      转向衰减到90%
```

4. **调整衰减参数**（如果需要）：
```bash
# 更早开始衰减（如6m/s）
ros2 param set /pp_param_tuner start_scale_speed 6.0

# 更激进的衰减（降到70%）
ros2 param set /pp_param_tuner steer_downscale_factor 0.70
```

---

## 📈 实时监控

### 查看所有参数
```bash
ros2 param list /pp_param_tuner
```

### 查看特定参数
```bash
ros2 param get /pp_param_tuner lateral_error_gain
ros2 param get /pp_param_tuner start_scale_speed
```

### 使用rqt动态调参
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

---

## 🎯 优化目标

### 直线路段
- [ ] RMS误差 < 10cm
- [ ] 最大误差 < 20cm
- [ ] 无明显抖动（转向变化率 < 3 rad/s）

### 弯道路段
- [ ] RMS误差 < 30cm（可以放宽要求）
- [ ] 不冲出赛道
- [ ] 平滑转向

### 高速性能
- [ ] 在8m/s时无侧滑
- [ ] 平稳加速/减速
- [ ] 稳定跟踪

---

## 🔍 下次测试

录制新的bag进行对比：
```bash
cd /home/nuc/RallyCore/bag
ros2 bag record -o pp_tuning_improved \
  /odometry/filtered \
  /drive \
  /pp/current_trajectory \
  /pp/lookahead_point
```

然后运行分析：
```bash
python3 /home/nuc/RallyCore/src/f1tenth_system/scripts/analyze_pp_bag.py
```

---

## 💡 调参技巧

1. **一次只改一个参数**，观察效果
2. **从小幅度开始**，逐步增加
3. **关注日志中的cte值**，应该稳定在0.05m以下
4. **在不同速度下测试**，确保全速域稳定
5. **记录每次调整的效果**，便于回溯

---

## ⚠️ 注意事项

- `lateral_error_gain` 过大可能导致抖动
- `heading_error_gain` 过大可能导致超调
- `min_lookahead` 过小会使车辆反应过激
- 高速时转向衰减太激进会导致转弯不足

---

## 🚀 快速启动命令

```bash
# 启动pure pursuit调参节点
ros2 run f1tenth_system pp_param_tuner

# 另一个终端：录制bag
cd /home/nuc/RallyCore/bag
ros2 bag record -o test_run \
  /odometry/filtered /drive /pp/current_trajectory /pp/lookahead_point

# 测试完成后分析
python3 /home/nuc/RallyCore/src/f1tenth_system/scripts/analyze_pp_bag.py
```
