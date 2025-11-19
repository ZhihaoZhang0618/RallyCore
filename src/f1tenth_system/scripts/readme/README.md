# Phase 2 文档索引

> **升级**：三份核心指南 + 新 Pure Pursuit 调参入口 | **大小**：45.5 KB (~50 KB) | **结构**：清晰导航 + PP 调参入口

---

## 📚 三个核心文档

### 1️⃣ ./PHASE2_GUIDE.md · 快速启用
- 面向：一线驾驶 & B 端调试
- 内容：5 分钟上手、完整流程、常见问题、参数配置表、部署步骤、数据分析样例、新增“PP 参数调试流程”
- 规模：351 行 | 10.6 KB

### 2️⃣ ./TECHNICAL_GUIDE.md · 深度参考
- 面向：算法/架构/控制开发
- 内容：系统架构、物理模型、四大核心类、Pure Pursuit 增强、`PPTuningNode` 结构与调参建议、扩展与单测
- 规模：700 行 | 22 KB

### 3️⃣ ./DEPLOYMENT_GUIDE.md · 运维工具
- 面向：现场部署 / 运维
- 内容：部署 checklist、编译、首次运行（含“PP 预调试”步骤）、监控与性能调优、命令速查
- 规模：484 行 | 13 KB

---

## 🔧 Pure Pursuit Tuner（PP 参数调试专用）
- 脚本：`pp_param_tuner.py`
- 作用：在做电流/加速度标定前，先把 Pure Pursuit 参数调顺，确保任意速度都紧贴轨迹、不再"扭来扭去"或拐弯半径过大
- 输出：`/drive` topic（兼容 V2 bringup 和 Nav2）
- 快速使用：
  ```bash
  # 🆕 使用 V2 bringup（推荐）
  ros2 launch f1tenth_system base_orin_livox_bringup_v2.launch.py
  
  # 默认启动（1m/s）
  ros2 run f1tenth_system pp_param_tuner
  
  # 指定初始速度
  ros2 run f1tenth_system pp_param_tuner --ros-args -p target_speed:=2.0
  
  # 实时调速
  ros2 param set /pp_param_tuner target_speed 3.0
  
  # rqt 图形化调参
  ros2 run rqt_reconfigure rqt_reconfigure
  ```
- 文档索引：
  - `./PHASE2_GUIDE.md` ➜ “PP 参数调试流程” 小节
  - `./TECHNICAL_GUIDE.md` ➜ `PPTuningNode` 架构 + 调参建议
  - `./DEPLOYMENT_GUIDE.md` ➜ 首次运行中的 “PP 预调试” 步骤

---

## 🎯 快速索引

| 目标 | 文档 | 章节 |
|------|------|------|
| 立即上手 | ./PHASE2_GUIDE.md | 快速开始 + 部署 |
| 调整标定参数 | ./PHASE2_GUIDE.md | 参数配置表 |
| PP 调参 / 稳定性 | ./PHASE2_GUIDE.md & ./TECHNICAL_GUIDE.md | 新增 PP 章节 |
| 深入理解代码 | ./TECHNICAL_GUIDE.md | 架构 / 代码详解 |
| 运维 / 首次部署 | ./DEPLOYMENT_GUIDE.md | 部署步骤 + PP 预调试 |
| 常见问题排查 | ./PHASE2_GUIDE.md ➜ 快速版 | ./TECHNICAL_GUIDE.md ➜ 技术版 |

---

## ✅ 推荐阅读顺序
1. **第一次使用 (~30 min)**：`./PHASE2_GUIDE.md` ➜ `./DEPLOYMENT_GUIDE.md` ➜ `pp_param_tuner`
2. **开发/调参**：`./TECHNICAL_GUIDE.md` ➜ `./PHASE2_GUIDE.md` ➜ `pp_param_tuner`
3. **现场运维**：`./DEPLOYMENT_GUIDE.md` ➜ 快速命令速查

---

## 💡 使用建议
- 先跑 `pp_param_tuner.py` 把 Pure Pursuit 调稳，再运行 `current_acc_calib.py`
- 推荐使用 `rqt_reconfigure` 图形化界面调参，实时拖动滑块看效果
- 关注日志里的 `cte_rms`（横向误差）和 `heading_rms`（航向误差）指标
- RViz 中添加 `/pp/current_trajectory` 可视化参考轨迹
- 不必一次性通读所有文档，按照角色 + 快速索引跳转即可

---

文档 & 调参工具已同步更新，PP 调参 + 标定流程一站式到位！

---

## 🏗️ 硬件 Bringup 架构说明

### V2 架构（推荐 - 集成式控制）
**Launch 文件：** `base_orin_livox_bringup_v2.launch.py`

**架构图：**
```
RC遥控器 ─────┐
Nav2导航 ─────┤
PP调参   ─────├──▶ joystick_control_v2.py ──▶ VESC驱动
标定程序 ─────┘     (内置命令仲裁逻辑)
```

**特点：**
- ✅ 单一控制节点，内置命令仲裁
- ✅ 支持 3 种 ESC 模式：速度/电流/占空比
- ✅ 遥控器实时切换控制源（teleop/nav）
- ✅ 集成标定模式（channel 10 开关）
- ✅ 输入 topic：`/drive` 和 `/calib/ackermann_cmd`
- ✅ 输出 topic：`/ackermann_drive`

**适用场景：**
- 电流加速度标定
- PP 参数调试
- Nav2 导航
- 遥控器直接控制

### V1 架构（传统 - Mux分离式）
**Launch 文件：** `base_orin_livox_bringup.launch.py`

**架构图：**
```
RC遥控器 ──▶ joystick_control.py ──┐
                                    ├──▶ ackermann_mux ──▶ VESC驱动
Nav2导航 ──────────────────────────┘    (优先级仲裁)
```

**特点：**
- 使用独立的 `ackermann_mux` 节点处理优先级
- 多 topic 输入架构（`/teleop`, `/drive` 等）
- 优先级通过 YAML 配置
- 不直接支持电流控制模式

**适用场景：**
- 传统 Nav2 导航（仅速度控制）
- 需要明确配置优先级的场景
- 兼容旧版代码


