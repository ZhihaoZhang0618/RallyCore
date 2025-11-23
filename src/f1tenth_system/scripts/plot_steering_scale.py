#!/usr/bin/env python3
"""可视化高速转向衰减曲线"""

import numpy as np
import matplotlib.pyplot as plt

def steering_scale(speed, start_speed=7.0, end_speed=8.0, downscale=0.80):
    """计算转向衰减系数"""
    if speed < start_speed:
        return 1.0
    elif speed >= end_speed:
        return downscale
    else:
        alpha = (speed - start_speed) / (end_speed - start_speed)
        return 1.0 - alpha * (1.0 - downscale)

# 创建速度数组
speeds = np.linspace(0, 10, 200)

# 计算不同配置下的衰减曲线
scale_default = np.array([steering_scale(v, 7.0, 8.0, 0.80) for v in speeds])
scale_early = np.array([steering_scale(v, 5.0, 7.0, 0.80) for v in speeds])
scale_aggressive = np.array([steering_scale(v, 7.0, 8.0, 0.60) for v in speeds])

# 绘图
plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.plot(speeds, scale_default * 100, 'b-', linewidth=2, label='Default (7→8m/s, 80%)')
plt.plot(speeds, scale_early * 100, 'g--', linewidth=2, label='Early (5→7m/s, 80%)')
plt.plot(speeds, scale_aggressive * 100, 'r-.', linewidth=2, label='Aggressive (7→8m/s, 60%)')
plt.axvline(x=5.0, color='orange', linestyle=':', alpha=0.5, label='Current slip speed')
plt.axhline(y=100, color='k', linestyle='-', alpha=0.3)
plt.axhline(y=80, color='k', linestyle='--', alpha=0.3)
plt.xlabel('Speed (m/s)', fontsize=12)
plt.ylabel('Steering Limit (%)', fontsize=12)
plt.title('High-Speed Steering Scaling Comparison', fontsize=14, fontweight='bold')
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)
plt.xlim(0, 10)
plt.ylim(50, 105)

# 第二个子图：转向角对比
max_steering = 0.35  # rad, 20度
plt.subplot(2, 1, 2)
steer_default = scale_default * max_steering
steer_early = scale_early * max_steering
steer_aggressive = scale_aggressive * max_steering

plt.plot(speeds, np.degrees(steer_default), 'b-', linewidth=2, label='Default')
plt.plot(speeds, np.degrees(steer_early), 'g--', linewidth=2, label='Early')
plt.plot(speeds, np.degrees(steer_aggressive), 'r-.', linewidth=2, label='Aggressive')
plt.axvline(x=5.0, color='orange', linestyle=':', alpha=0.5, label='Current slip speed')
plt.axhline(y=np.degrees(max_steering), color='k', linestyle='-', alpha=0.3)
plt.xlabel('Speed (m/s)', fontsize=12)
plt.ylabel('Max Steering Angle (degrees)', fontsize=12)
plt.title('Maximum Steering Angle vs Speed', fontsize=14, fontweight='bold')
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)
plt.xlim(0, 10)

plt.tight_layout()
plt.savefig('/home/nuc/RallyCore/bag/steering_scale_curves.png', dpi=150, bbox_inches='tight')
print("✓ Steering scale curves saved to: /home/nuc/RallyCore/bag/steering_scale_curves.png")

# 打印关键速度点的数值
print("\n📊 Steering Scale at Key Speeds:")
print("=" * 60)
for config_name, scale_func in [
    ("Default (7→8m/s, 80%)", lambda v: steering_scale(v, 7.0, 8.0, 0.80)),
    ("Early (5→7m/s, 80%)", lambda v: steering_scale(v, 5.0, 7.0, 0.80)),
    ("Aggressive (7→8m/s, 60%)", lambda v: steering_scale(v, 7.0, 8.0, 0.60))
]:
    print(f"\n{config_name}:")
    for speed in [3.0, 5.0, 6.0, 7.0, 7.5, 8.0, 9.0]:
        scale = scale_func(speed)
        angle = np.degrees(scale * max_steering)
        print(f"  {speed:4.1f} m/s → {scale*100:5.1f}% → {angle:5.2f}°")
