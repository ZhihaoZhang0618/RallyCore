#!/usr/bin/env python3
"""分析Pure Pursuit bag文件，重点关注直线和弯道的跟踪性能"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3

def quaternion_to_yaw(qx, qy, qz, qw):
    """从四元数计算yaw角"""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def load_bag_data(bag_path):
    """从bag文件加载数据"""
    db_path = Path(bag_path) / f"{Path(bag_path).name}_0.db3"
    
    if not db_path.exists():
        print(f"错误: 找不到 {db_path}")
        return None
    
    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()
    
    # 获取topic信息
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: {'id': row[0], 'type': row[2]} for row in cursor.fetchall()}
    
    print("📦 Bag文件包含的topics:")
    for name, info in topics.items():
        cursor.execute("SELECT COUNT(*) FROM messages WHERE topic_id = ?", (info['id'],))
        count = cursor.fetchone()[0]
        print(f"  - {name} ({info['type']}) - {count} 条消息")
    
    data = {
        'odom': {'t': [], 'x': [], 'y': [], 'yaw': [], 'vx': [], 'vy': []},
        'traj': {'t': [], 'x': [], 'y': []},
        'lookahead': {'t': [], 'x': [], 'y': []},
        'drive': {'t': [], 'steering': [], 'speed': []},
    }
    
    # 加载里程计数据
    if '/odometry/filtered' in topics:
        topic_id = topics['/odometry/filtered']['id']
        msg_type = get_message(topics['/odometry/filtered']['type'])
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
        
        for row in cursor.fetchall():
            try:
                timestamp = row[0] / 1e9
                msg = deserialize_message(row[1], msg_type)
                
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                yaw = quaternion_to_yaw(qx, qy, qz, qw)
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                
                data['odom']['t'].append(timestamp)
                data['odom']['x'].append(x)
                data['odom']['y'].append(y)
                data['odom']['yaw'].append(yaw)
                data['odom']['vx'].append(vx)
                data['odom']['vy'].append(vy)
            except Exception as e:
                pass
    
    # 加载参考轨迹
    if '/pp/current_trajectory' in topics:
        topic_id = topics['/pp/current_trajectory']['id']
        msg_type = get_message(topics['/pp/current_trajectory']['type'])
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? LIMIT 1", (topic_id,))
        
        row = cursor.fetchone()
        if row:
            try:
                timestamp = row[0] / 1e9
                msg = deserialize_message(row[1], msg_type)
                for pose in msg.poses:
                    data['traj']['x'].append(pose.pose.position.x)
                    data['traj']['y'].append(pose.pose.position.y)
                data['traj']['t'] = timestamp
            except Exception as e:
                print(f"轨迹加载失败: {e}")
    
    # 加载lookahead点
    if '/pp/lookahead_point' in topics:
        topic_id = topics['/pp/lookahead_point']['id']
        msg_type = get_message(topics['/pp/lookahead_point']['type'])
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
        
        for row in cursor.fetchall():
            try:
                timestamp = row[0] / 1e9
                msg = deserialize_message(row[1], msg_type)
                x = msg.point.x
                y = msg.point.y
                data['lookahead']['t'].append(timestamp)
                data['lookahead']['x'].append(x)
                data['lookahead']['y'].append(y)
            except:
                pass
    
    # 加载控制指令
    if '/drive' in topics:
        topic_id = topics['/drive']['id']
        msg_type = get_message(topics['/drive']['type'])
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
        
        for row in cursor.fetchall():
            try:
                timestamp = row[0] / 1e9
                msg = deserialize_message(row[1], msg_type)
                steering = msg.drive.steering_angle
                speed = msg.drive.speed
                data['drive']['t'].append(timestamp)
                data['drive']['steering'].append(steering)
                data['drive']['speed'].append(speed)
            except:
                pass
    
    conn.close()
    
    # 转换为numpy数组
    for key in data:
        for subkey in data[key]:
            if isinstance(data[key][subkey], list):
                data[key][subkey] = np.array(data[key][subkey])
    
    return data

def detect_straight_sections(x, y, yaw, window=20):
    """检测直线路段（航向角变化小的区域）"""
    straight_mask = np.zeros(len(x), dtype=bool)
    
    for i in range(window, len(x) - window):
        # 计算窗口内的航向角变化
        yaw_window = yaw[i-window:i+window]
        yaw_diff = np.abs(np.diff(yaw_window))
        # 处理角度跳变
        yaw_diff[yaw_diff > np.pi] = 2*np.pi - yaw_diff[yaw_diff > np.pi]
        
        # 如果航向角变化很小，认为是直线
        if np.max(yaw_diff) < 0.05:  # 约3度
            straight_mask[i] = True
    
    return straight_mask

def compute_cross_track_error(odom_x, odom_y, traj_x, traj_y):
    """计算横向误差（到轨迹的最近距离）"""
    if len(traj_x) == 0 or len(odom_x) == 0:
        return np.array([])
    
    cte = []
    traj_points = np.column_stack([traj_x, traj_y])
    
    for i in range(len(odom_x)):
        pos = np.array([odom_x[i], odom_y[i]])
        distances = np.linalg.norm(traj_points - pos, axis=1)
        cte.append(np.min(distances))
    
    return np.array(cte)

def analyze_bag(bag_path):
    """分析bag文件"""
    print(f"\n🔍 分析bag文件: {bag_path}\n")
    
    data = load_bag_data(bag_path)
    if data is None:
        return
    
    # 检查数据
    n_odom = len(data['odom']['t'])
    n_lookahead = len(data['lookahead']['t'])
    n_drive = len(data['drive']['t'])
    
    print(f"\n📊 数据统计:")
    print(f"  里程计消息: {n_odom}")
    print(f"  Lookahead点: {n_lookahead}")
    print(f"  控制指令: {n_drive}")
    
    if n_odom == 0:
        print("❌ 没有里程计数据!")
        return
    
    # 检测直线和弯道
    straight_mask = detect_straight_sections(
        data['odom']['x'], data['odom']['y'], data['odom']['yaw']
    )
    
    print(f"\n📏 轨迹分析:")
    print(f"  总点数: {len(straight_mask)}")
    print(f"  直线路段: {np.sum(straight_mask)} ({100*np.sum(straight_mask)/len(straight_mask):.1f}%)")
    print(f"  弯道路段: {np.sum(~straight_mask)} ({100*np.sum(~straight_mask)/len(straight_mask):.1f}%)")
    
    # 计算速度
    speed = np.sqrt(data['odom']['vx']**2 + data['odom']['vy']**2)
    print(f"\n🚗 速度统计:")
    print(f"  平均速度: {np.mean(speed):.2f} m/s")
    print(f"  最大速度: {np.max(speed):.2f} m/s")
    print(f"  最小速度: {np.min(speed):.2f} m/s")
    
    # 计算横向误差
    cte = None
    if len(data['traj']['x']) > 0:
        # 使用参考轨迹
        cte = compute_cross_track_error(
            data['odom']['x'], data['odom']['y'],
            data['traj']['x'], data['traj']['y']
        )
    
    if cte is not None and len(cte) > 0:
        print(f"\n📐 横向误差 (Cross Track Error):")
        print(f"  全程 - 平均: {np.mean(cte)*100:.2f}cm, 最大: {np.max(cte)*100:.2f}cm, RMS: {np.sqrt(np.mean(cte**2))*100:.2f}cm")
        
        if np.sum(straight_mask) > 0:
            cte_straight = cte[straight_mask]
            print(f"  直线 - 平均: {np.mean(cte_straight)*100:.2f}cm, 最大: {np.max(cte_straight)*100:.2f}cm, RMS: {np.sqrt(np.mean(cte_straight**2))*100:.2f}cm")
        
        if np.sum(~straight_mask) > 0:
            cte_turn = cte[~straight_mask]
            print(f"  弯道 - 平均: {np.mean(cte_turn)*100:.2f}cm, 最大: {np.max(cte_turn)*100:.2f}cm, RMS: {np.sqrt(np.mean(cte_turn**2))*100:.2f}cm")
    
    # 分析控制指令
    if n_drive > 0:
        print(f"\n🎮 控制指令分析:")
        print(f"  平均速度: {np.mean(data['drive']['speed']):.2f} m/s")
        print(f"  速度范围: {np.min(data['drive']['speed']):.2f} - {np.max(data['drive']['speed']):.2f} m/s")
        print(f"  平均转向角: {np.mean(np.abs(data['drive']['steering'])):.3f} rad ({np.degrees(np.mean(np.abs(data['drive']['steering']))):.1f}°)")
        print(f"  最大转向角: {np.max(np.abs(data['drive']['steering'])):.3f} rad ({np.degrees(np.max(np.abs(data['drive']['steering']))):.1f}°)")
        
        # 转向角变化率（抖动检测）
        if len(data['drive']['steering']) > 1:
            steering_rate = np.abs(np.diff(data['drive']['steering'])) / np.diff(data['drive']['t'])
            print(f"  转向变化率: 平均 {np.mean(steering_rate):.2f} rad/s, 最大 {np.max(steering_rate):.2f} rad/s")
    
    # 绘制结果
    plot_results(data, straight_mask, cte, speed)
    
    # 给出调优建议
    print(f"\n\n💡 参数调优建议:")
    if cte is not None and len(cte) > 0:
        cte_straight = cte[straight_mask] if np.sum(straight_mask) > 0 else cte
        cte_rms_straight = np.sqrt(np.mean(cte_straight**2)) * 100
        
        if cte_rms_straight > 10:
            print(f"  ⚠️  直线路段RMS误差 {cte_rms_straight:.1f}cm 较大，建议:")
            print(f"     - 增加 lateral_error_gain (当前应该是1.0，建议试试 1.2-1.5)")
            print(f"     - 减小 min_lookahead (当前0.3，可以试试 0.2-0.25)")
            print(f"     - 增加 heading_error_gain (当前0.2，可以试试 0.3-0.4)")
        elif cte_rms_straight > 5:
            print(f"  ✓ 直线路段RMS误差 {cte_rms_straight:.1f}cm 中等，可以微调:")
            print(f"     - 尝试增加 lateral_error_gain 到 1.1-1.3")
        else:
            print(f"  ✓✓ 直线路段RMS误差 {cte_rms_straight:.1f}cm 很好!")
    
    if n_drive > 0:
        steering_rate = np.abs(np.diff(data['drive']['steering'])) / np.diff(data['drive']['t'])
        if np.mean(steering_rate) > 5.0:
            print(f"  ⚠️  转向变化率 {np.mean(steering_rate):.1f} rad/s 较大，车辆可能抖动")
            print(f"     - 考虑增加转向平滑 (如果有的话)")
    
    print(f"\n  📌 高速转向限制:")
    print(f"     - 当前在5.5m/s目标速度下，如果实际速度>5m/s时发生侧滑")
    print(f"     - 建议添加速度依赖的转向角限制")
    print(f"     - 参考公式: max_steering_at_speed = max_steering * (start_scale_speed / current_speed)")
    print(f"     - 例如: 7m/s时限制为70%, 8m/s时限制为80%")

def plot_results(data, straight_mask, cte, speed):
    """绘制分析结果"""
    fig = plt.figure(figsize=(18, 12))
    
    # 1. 轨迹图
    ax1 = plt.subplot(3, 3, 1)
    if len(data['traj']['x']) > 0:
        ax1.plot(data['traj']['x'], data['traj']['y'], 'k--', linewidth=2, alpha=0.6, label='参考轨迹')
    ax1.plot(data['odom']['x'], data['odom']['y'], 'b-', linewidth=0.5, alpha=0.5, label='实际轨迹')
    ax1.scatter(data['odom']['x'][straight_mask], data['odom']['y'][straight_mask], 
                c='green', s=3, alpha=0.4, label='直线路段')
    ax1.scatter(data['odom']['x'][~straight_mask], data['odom']['y'][~straight_mask], 
                c='orange', s=3, alpha=0.4, label='弯道路段')
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_title('轨迹 - 直线vs弯道', fontsize=11, fontweight='bold')
    ax1.legend(fontsize=8)
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # 2. 横向误差时间序列
    if cte is not None and len(cte) > 0:
        ax2 = plt.subplot(3, 3, 2)
        t_rel = data['odom']['t'] - data['odom']['t'][0]
        ax2.plot(t_rel, cte * 100, 'b-', linewidth=0.5, alpha=0.3)
        ax2.plot(t_rel[straight_mask], cte[straight_mask] * 100, 'g.', 
                markersize=2, alpha=0.5, label='直线')
        ax2.plot(t_rel[~straight_mask], cte[~straight_mask] * 100, 'orange', 
                marker='.', markersize=2, linestyle='', alpha=0.5, label='弯道')
        ax2.set_xlabel('时间 (s)', fontsize=10)
        ax2.set_ylabel('横向误差 (cm)', fontsize=10)
        ax2.set_title('横向误差随时间变化', fontsize=11, fontweight='bold')
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)
        
        # 3. 横向误差分布
        ax3 = plt.subplot(3, 3, 3)
        if np.sum(straight_mask) > 0:
            ax3.hist(cte[straight_mask] * 100, bins=40, alpha=0.6, color='green', label='直线')
        if np.sum(~straight_mask) > 0:
            ax3.hist(cte[~straight_mask] * 100, bins=40, alpha=0.6, color='orange', label='弯道')
        ax3.set_xlabel('横向误差 (cm)', fontsize=10)
        ax3.set_ylabel('频次', fontsize=10)
        ax3.set_title('横向误差分布', fontsize=11, fontweight='bold')
        ax3.legend(fontsize=8)
        ax3.grid(True, alpha=0.3)
    
    # 4. 速度时间序列 (从里程计)
    ax4 = plt.subplot(3, 3, 4)
    t_rel = data['odom']['t'] - data['odom']['t'][0]
    ax4.plot(t_rel, speed, 'b-', linewidth=0.8)
    ax4.axhline(y=5.0, color='r', linestyle='--', linewidth=1, label='5m/s (侧滑阈值)')
    ax4.set_xlabel('时间 (s)', fontsize=10)
    ax4.set_ylabel('速度 (m/s)', fontsize=10)
    ax4.set_title('速度随时间变化 (里程计)', fontsize=11, fontweight='bold')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    
    # 5. 转向角时间序列
    if len(data['drive']['t']) > 0:
        ax5 = plt.subplot(3, 3, 5)
        t_rel_drive = data['drive']['t'] - data['drive']['t'][0]
        ax5.plot(t_rel_drive, np.degrees(data['drive']['steering']), 'r-', linewidth=0.8)
        ax5.axhline(y=np.degrees(0.35), color='k', linestyle='--', linewidth=1, alpha=0.5, label='最大转向角')
        ax5.axhline(y=-np.degrees(0.35), color='k', linestyle='--', linewidth=1, alpha=0.5)
        ax5.set_xlabel('时间 (s)', fontsize=10)
        ax5.set_ylabel('转向角 (度)', fontsize=10)
        ax5.set_title('转向角随时间变化', fontsize=11, fontweight='bold')
        ax5.legend(fontsize=8)
        ax5.grid(True, alpha=0.3)
        
        # 6. 转向角分布
        ax6 = plt.subplot(3, 3, 6)
        ax6.hist(np.degrees(data['drive']['steering']), bins=50, alpha=0.7, color='red', edgecolor='black')
        ax6.set_xlabel('转向角 (度)', fontsize=10)
        ax6.set_ylabel('频次', fontsize=10)
        ax6.set_title('转向角分布', fontsize=11, fontweight='bold')
        ax6.grid(True, alpha=0.3)
    
    # 7. 速度vs横向误差
    if cte is not None and len(cte) > 0:
        ax7 = plt.subplot(3, 3, 7)
        scatter = ax7.scatter(speed, cte * 100, c=straight_mask.astype(float), 
                             s=2, alpha=0.3, cmap='RdYlGn')
        ax7.set_xlabel('速度 (m/s)', fontsize=10)
        ax7.set_ylabel('横向误差 (cm)', fontsize=10)
        ax7.set_title('速度 vs 横向误差', fontsize=11, fontweight='bold')
        ax7.grid(True, alpha=0.3)
        cbar = plt.colorbar(scatter, ax=ax7)
        cbar.set_label('直线(1) / 弯道(0)', fontsize=8)
    
    # 8. 速度分布（热力图）
    ax8 = plt.subplot(3, 3, 8)
    speed_bins = np.linspace(0, np.max(speed), 30)
    if cte is not None and len(cte) > 0:
        cte_mean = []
        for i in range(len(speed_bins)-1):
            mask = (speed >= speed_bins[i]) & (speed < speed_bins[i+1])
            if np.sum(mask) > 0:
                cte_mean.append(np.mean(cte[mask]) * 100)
            else:
                cte_mean.append(0)
        ax8.bar(speed_bins[:-1], cte_mean, width=np.diff(speed_bins), 
               alpha=0.7, color='steelblue', edgecolor='black')
        ax8.set_xlabel('速度 (m/s)', fontsize=10)
        ax8.set_ylabel('平均横向误差 (cm)', fontsize=10)
        ax8.set_title('不同速度下的平均横向误差', fontsize=11, fontweight='bold')
        ax8.grid(True, alpha=0.3, axis='y')
    
    # 9. 转向率（抖动分析）
    if len(data['drive']['t']) > 1:
        ax9 = plt.subplot(3, 3, 9)
        steering_rate = np.abs(np.diff(data['drive']['steering'])) / np.diff(data['drive']['t'])
        t_steering_rate = (data['drive']['t'][:-1] + data['drive']['t'][1:]) / 2 - data['drive']['t'][0]
        ax9.plot(t_steering_rate, steering_rate, 'purple', linewidth=0.6, alpha=0.6)
        ax9.axhline(y=5.0, color='r', linestyle='--', linewidth=1, label='抖动阈值')
        ax9.set_xlabel('时间 (s)', fontsize=10)
        ax9.set_ylabel('转向变化率 (rad/s)', fontsize=10)
        ax9.set_title('转向变化率 (抖动检测)', fontsize=11, fontweight='bold')
        ax9.legend(fontsize=8)
        ax9.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/nuc/RallyCore/bag/pp_tuning_analysis.png', dpi=150, bbox_inches='tight')
    print(f"\n📊 分析图表已保存到: /home/nuc/RallyCore/bag/pp_tuning_analysis.png")
    plt.show()

if __name__ == '__main__':
    bag_path = '/home/nuc/RallyCore/bag/pp_tuning_test'
    analyze_bag(bag_path)
