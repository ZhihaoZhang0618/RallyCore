import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
import time
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.timer import Timer
import tf2_ros
import geometry_msgs.msg
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from f_sgp_bgk import TraversabilityAnalyzer
from planner import GridMap, indexToPos, PenalTrajOpt
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, ColorRGBA, String

class GogoPlannerNode(Node):
    def __init__(self):
        super().__init__('gogo_planner_node')

        # 设置话题
        self.cloud_topic = '/cloud_registered_body'  # 点云话题
        self.odom_topic = '/aft_mapped_to_init'  # 里程计话题
        self.waypoint_topic = '/goal_pose'  # RViz2 发布的目标点话题

        # 创建订阅器
        self.sph_pcl_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.elevation_cb,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10
        )

        self.waypoint_sub = self.create_subscription(
            PoseStamped,  # 假设 RViz2 发布的是 PoseStamped 消息
            self.waypoint_topic,
            self.waypoint_cb,
            10
        )

        # 创建发布器
        self.traversability_pcl_pub = self.create_publisher(PointCloud2, "color_pcl", 10)
        self.path_pub = self.create_publisher(Path, "planned_path", 10)  # 路径发布器
        # 新增发布器：用于发布子目标点
        self.subgoal_pub = self.create_publisher(PointStamped, 'subgoal_point', 10)
        # 状态发布器
        self.state_pub = self.create_publisher(String, "planner_state", 10)

        # 定时器，定期调用处理函数
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每0.1秒执行一次

        # 初始化变量
        self.latest_pcl = None
        self.latest_odom = None
        self.global_pose = None
        self.waypoint = None  # 全局目标点

        # 用于任务一的子目标点，当全局目标在地图外时采用
        self.current_subgoal = None
        # 用于任务二：规划完成后保存全局轨迹，不重复规划
        self.global_trajps = None
        self.current_target = None  # 当前规划目标（可能是全局目标或子目标）

        # 状态机（IDLE, PLANNING, EXECUTING, COMPLETED）
        self.state = "IDLE"

        # 创建tf2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.analyzer = TraversabilityAnalyzer(config_path="../config/params.yaml")

        # 新增标志位，控制地图更新
        self.update_map_flag = False
        
        self.last_task_type=None

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_path(self, trajps):
        if trajps is None:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "camera_init"
        for point in trajps:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = "camera_init"
            # 轨迹点在局部坐标系下，加上机器人全局位姿即可转换为全局坐标系
            pose_stamped.pose.position.x = point[0] 
            pose_stamped.pose.position.y = point[1] 
            # 查询该点高度
            result = self.analyzer.query(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            if result["mean"] is None:
                height = self.global_pose[2]
            else:
                height = result["mean"]
            pose_stamped.pose.position.z = height + 0.2

            # 设置姿态：将 yaw 转换为四元数
            yaw = point[2]
            quat = quaternion_from_euler(0.0, 0.0, yaw)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)
        # self.get_logger().info("Published Path message.")

    def odom_cb(self, msg):
        """处理里程计数据回调函数"""
        self.latest_odom = msg
        # self.get_logger().info('Received Odometry message')

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        try:
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        except Exception as ex:
            self.get_logger().warn(f'Error converting quaternion to Euler angles: {ex}')
            return

        self.global_pose = [position.x, position.y, position.z, roll, pitch, yaw]

    def elevation_cb(self, msg):
        """处理点云数据回调函数"""
        self.latest_pcl = msg
        # self.get_logger().info('Received PointCloud2 message')

    def waypoint_cb(self, msg):
        """处理目标点数据回调函数"""
        position = msg.pose.position
        orientation = msg.pose.orientation

        try:
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        except Exception as ex:
            self.get_logger().warn(f'Error converting quaternion to Euler angles: {ex}')
            return

        self.waypoint = [position.x, position.y, position.z, roll, pitch, yaw]
        self.get_logger().info(f'Received Waypoint: {self.waypoint}')
        # 每次收到新目标点时，重置规划状态和子目标
        self.state = "PLANNING"
        self.global_trajps = None
        self.current_subgoal = None
        self.current_target = None
        self.publish_state()
        # 设置标志位，表示需要更新地图
        self.update_map_flag = True

    def timer_callback(self):
        """定时器回调函数，用于处理点云和里程计数据"""
        if self.latest_pcl and self.latest_odom: #and self.waypoint:
            self.process_data(self.latest_pcl, self.latest_odom)
        else:
            self.get_logger().info('Waiting for data...')

    def generate_query_grid(self, pose, grid_size=5.0, resolution=0.2):
        """
        生成以当前pose为中心的查询网格
        pose: 机器人位姿 [x, y, z, roll, pitch, yaw]
        grid_size: 网格范围（以米为单位）
        resolution: 网格分辨率（默认 0.2m）
        返回：查询网格点列表 (N, 2) -> (x, y)
        """
        x_center, y_center = pose[:2]
        num_points_per_axis = int(grid_size / resolution)
        x_range = np.linspace(x_center - grid_size / 2, x_center + grid_size / 2, num_points_per_axis)
        y_range = np.linspace(y_center - grid_size / 2, y_center + grid_size / 2, num_points_per_axis)
        xx, yy = np.meshgrid(x_range, y_range)
        query_points = np.column_stack((xx.ravel(), yy.ravel()))
        return query_points

    def generate_costmap(self, query_points, traversability_values, grid_size_x, grid_size_y, resolution):
        """
        生成代价地图，并在内部区域根据可通行性赋值（非内部边界保持为0，内部区域满足条件的置为1）
        query_points: (N, 2) 查询点坐标
        traversability_values: (N,) 对应的可通行性值
        grid_size_x/y: 地图尺寸
        resolution: 地图分辨率
        返回：costmap (numpy数组)
        """
        num_x = int(grid_size_x / resolution)
        num_y = int(grid_size_y / resolution)
        costmap = np.zeros((num_x, num_y))
        x_min, y_min = query_points[:, 0].min(), query_points[:, 1].min()
        for i, (x, y) in enumerate(query_points):
            idx_x = int((x - x_min) / resolution)
            idx_y = int((y - y_min) / resolution)
            # 只填充内部区域（不改变地图边界值，便于后续选取边界上的子目标）
            if 0 < idx_x < num_x - 1 and 0 < idx_y < num_y - 1:
                if traversability_values[i] > 0.8:
                    costmap[idx_x, idx_y] = 1
        return costmap

    def select_subgoal(self, costmap, grid_map_parms, global_pose, waypoint, neighborhood_size=1, radius=1.5, num_samples=30):
        """
        通过圆形采样选取子目标点：在圆形区域内均匀采样若干点，筛选出符合可通行性条件的点，
        并选择最接近全局目标点的子目标点。
        如果没有符合条件的点，则选择costmap值最小的点作为子目标。
        返回：子目标点 [x, y, z, roll, pitch, yaw]
        """
        resolution = grid_map_parms["resolution"]
        map_size_x = grid_map_parms["map_size_x"]
        map_size_y = grid_map_parms["map_size_y"]
        num_x = int(map_size_x / resolution)
        num_y = int(map_size_y / resolution)

        # 计算局部地图的左下角坐标（局部地图以机器人global_pose为中心）
        x_min = global_pose[0] - map_size_x / 2.0
        y_min = global_pose[1] - map_size_y / 2.0

        best_score = float('inf')
        best_candidate = None

        # 计算圆形采样的点
        angles = np.linspace(0, 2 * np.pi, num_samples)  # 均匀分布的角度
        distances = np.full(num_samples, radius)  # 所有点的半径为1.2米

        for angle, dist in zip(angles, distances):
            # 计算采样点的坐标
            candidate_x = global_pose[0] + dist * np.cos(angle)
            candidate_y = global_pose[1] + dist * np.sin(angle)

            # 转换到costmap的索引
            i = int((candidate_x - x_min) / resolution)
            j = int((candidate_y - y_min) / resolution)

            if 0 <= i < num_x and 0 <= j < num_y:
                traversability = 1
                result = self.analyzer.query(candidate_x, candidate_y)
                if result["traversability"] is None:
                    traversability=1
                else:
                    traversability=result["traversability"]

                # 如果该点可通行，检查周围邻域
                if traversability <0.4:
                    is_valid = True
                    for di in range(-neighborhood_size, neighborhood_size + 1):
                        for dj in range(-neighborhood_size, neighborhood_size + 1):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < num_x and 0 <= nj < num_y:
                                if costmap[ni, nj] == 1:
                                    is_valid = False
                                    break
                        if not is_valid:
                            break

                    if is_valid:
                        # 计算当前候选点与目标点的距离
                        distance_to_goal = np.sqrt((candidate_x - waypoint[0])**2 + (candidate_y - waypoint[1])**2)
                        distance_from_robot = np.sqrt((candidate_x - global_pose[0])**2 + (candidate_y - global_pose[1])**2)

                        # 考虑距离权重
                        total_distance = distance_to_goal + distance_from_robot
                        if total_distance < best_score:
                            best_score = total_distance
                            best_candidate = [candidate_x, candidate_y]

        # 如果没有找到符合条件的子目标点，则选择costmap值最小的点
        # if best_candidate is None:
        #     best_val = float('inf')
        #     for i in range(num_x):
        #         for j in range(num_y):
        #             candidate_x = x_min + i * resolution
        #             candidate_y = y_min + j * resolution
        #             val = costmap[i, j]
        #             if val < best_val:
        #                 best_val = val
        #                 best_candidate = [candidate_x, candidate_y]

        if best_candidate is not None:
            # 计算朝向
            dx = best_candidate[0] - global_pose[0]
            dy = best_candidate[1] - global_pose[1]
            yaw = np.arctan2(dy, dx)
            return [best_candidate[0], best_candidate[1], waypoint[2], 0.0, 0.0, yaw]
        else:
            return waypoint

    def is_reached(self, target, threshold=0.5):
        """
        判断机器人是否到达目标点（target: [x, y, z, roll, pitch, yaw]）
        """
        dx = target[0] - self.global_pose[0]
        dy = target[1] - self.global_pose[1]
        return np.sqrt(dx**2 + dy**2) < threshold

    def process_data(self, pcl_msg, odom_msg):
        # 读取点云数据
        cloud_points = pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)
        
        local_points = [
            np.array([x, y, z]) for x, y, z in cloud_points 
            if x**2 + y**2 <= 25 and z < 1.5  # 半径 r=5，平方计算避免开根号
        ]
        
        local_points_np = np.array(local_points) if local_points else np.empty((0, 3))  # 防止空列表时报错

        self.analyzer.update_map(self.global_pose, local_points_np)

        # # 检查是否需要更新地图
        # if self.update_map_flag:
        #     self.analyzer.update_map(self.global_pose, local_points_np)
        #     self.update_map_flag = False  # 重置标志位

        # 生成查询网格（全局坐标系下）
        query_points = self.generate_query_grid(self.global_pose)
        traversability_values = []
        mean_xyz_values = []

        for x, y in query_points:
            result = self.analyzer.query(x, y)
            if result["traversability"] is None:
                traversability_values.append(0)
            else:
                traversability_values.append(result["traversability"])
            if result["mean"] is None:
                mean_xyz_values.append(0)
            else:
                mean_xyz_values.append([x, y, result["mean"]])

        # 构建并发布彩色点云（用于可视化）
        point_cloud_data = []
        for i, point in enumerate(mean_xyz_values):
            x, y, z = point
            color_value = traversability_values[i]
            color = ColorRGBA(r=float(color_value),
                              g=1.0 - float(color_value),
                              b=0.0,
                              a=1.0)
            point_cloud_data.append([x, y, z, color.r, color.g, color.b, color.a])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_init'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='a', offset=24, datatype=PointField.FLOAT32, count=1)
        ]
        pc2_msg = pc2.create_cloud(header, fields, point_cloud_data)
        self.traversability_pcl_pub.publish(pc2_msg)
        # return
        # 如果没有目标点则直接返回
        if self.waypoint is None:
            return

        # 地图参数
        grid_map_parms = {
            "map_size_x": 5.0,
            "map_size_y": 5.0,
            "resolution": 0.2
        }
        costmap = self.generate_costmap(query_points, traversability_values,
                                        grid_map_parms["map_size_x"],
                                        grid_map_parms["map_size_y"],
                                        grid_map_parms["resolution"])

        # 判断全局目标点是否在当前局部地图内部
        half_size_x = grid_map_parms["map_size_x"] / 2.0
        half_size_y = grid_map_parms["map_size_y"] / 2.0
        dx = abs(self.waypoint[0] - self.global_pose[0])
        dy = abs(self.waypoint[1] - self.global_pose[1])

        # 根据全局目标是否在局部地图内区分任务类型
        import math
        if math.sqrt(dx*dx+dy*dy)<2:
            target = self.waypoint
            task_type = "global"
        else:
            if self.current_subgoal is None or self.is_reached(self.current_subgoal):
                self.current_subgoal = self.select_subgoal(costmap, grid_map_parms, self.global_pose, self.waypoint)
                self.get_logger().info(f"Task One: Selected new subgoal: {self.current_subgoal}")
                self.get_logger().info(f"Task One: Selected new subgoal: {self.current_subgoal}")
                # 发布子目标点用于可视化
                subgoal_msg = PointStamped()
                subgoal_msg.header.stamp = self.get_clock().now().to_msg()
                subgoal_msg.header.frame_id = 'camera_init'
                subgoal_msg.point.x = self.current_subgoal[0]
                subgoal_msg.point.y = self.current_subgoal[1]
                # 查询该点高度
                result = self.analyzer.query(subgoal_msg.point.x , subgoal_msg.point.y )
                if result["mean"] is None:
                    height = self.global_pose[2]
                else:
                    height = result["mean"]
                subgoal_msg.point.z  = height + 0.2
                self.subgoal_pub.publish(subgoal_msg)
                self.update_map_flag = True
                target = self.current_subgoal
                start = [0, 0]
                end = [target[0] - self.global_pose[0], target[1] - self.global_pose[1]]
                # 使用A*进行初步路径规划
                map_obj = GridMap()
                map_obj.init(grid_map_parms)
                map_obj.setMap(costmap)
                startp = np.array([[start[0]], [start[1]]])  # 转换为 2x1 的 ndarray
                endp = np.array([[end[0]], [end[1]]])  # 转换为 2x1 的 ndarray
                astar_res = map_obj.astarPlan(startp, endp, 0.5)
                astar_path = astar_res[0]

                # 利用轨迹优化器生成平滑轨迹
                traj_opt = PenalTrajOpt()
                traj_opt.init(grid_map_parms)
                traj_opt.setMap(costmap)
                if traj_opt.plan([start[0], start[1], self.global_pose[5]],
                                  [end[0], end[1], target[5]],
                                  0.5):
                    traj = traj_opt.getTraj()
                    traj_time = traj.getTotalDuration()
                    trajps = []
                    for i in range(100):
                        t = i / 100.0 * traj_time
                        pos = traj.getPos(t)
                        trajps.append((pos[0]+self.global_pose[0],pos[1]+self.global_pose[1],pos[2]))
                    trajps = np.array(trajps)
                    self.global_trajps = trajps
                    self.current_target = target  # 保存当前规划目标
                    self.state = "EXECUTING"
                    self.publish_state()
                    self.publish_path(self.global_trajps)
                else:
                    self.get_logger().warn("Trajectory optimization failed.")
            else:
                self.get_logger().info(f"Task One: Continuing towards current subgoal: {self.current_subgoal}")
            target = self.current_subgoal
            task_type = "subgoal"

        if (self.is_reached(target)) and task_type == "subgoal":
            self.update_map_flag=True
            
        if(self.last_task_type=="subgoal" and task_type=="global"):
            start = [0, 0]
            end = [target[0] - self.global_pose[0], target[1] - self.global_pose[1]]
            # 使用A*进行初步路径规划
            map_obj = GridMap()
            map_obj.init(grid_map_parms)
            map_obj.setMap(costmap)
            startp = np.array([[start[0]], [start[1]]])  # 转换为 2x1 的 ndarray
            endp = np.array([[end[0]], [end[1]]])  # 转换为 2x1 的 ndarray
            astar_res = map_obj.astarPlan(startp, endp, 0.5)
            astar_path = astar_res[0]

            # 利用轨迹优化器生成平滑轨迹
            traj_opt = PenalTrajOpt()
            traj_opt.init(grid_map_parms)
            traj_opt.setMap(costmap)
            if traj_opt.plan([start[0], start[1], self.global_pose[5]],
                                [end[0], end[1], target[5]],
                                0.5):
                traj = traj_opt.getTraj()
                traj_time = traj.getTotalDuration()
                trajps = []
                for i in range(100):
                    t = i / 100.0 * traj_time
                    pos = traj.getPos(t)
                    trajps.append((pos[0]+self.global_pose[0],pos[1]+self.global_pose[1],pos[2]))
                trajps = np.array(trajps)
                self.global_trajps = trajps
                self.current_target = target  # 保存当前规划目标
                self.state = "EXECUTING"
                self.publish_state()
                self.publish_path(self.global_trajps)
            else:
                self.get_logger().warn("Trajectory optimization failed.")
                return
            
         
        self.last_task_type=task_type
        # 状态机逻辑
        print(task_type)
        if self.state == "PLANNING":
            # 如果目标已到达则不需要规划
            if self.is_reached(target):
                if task_type == "global":
                    self.state = "COMPLETED"
                    self.publish_state()
                    self.get_logger().info("Global goal reached, task completed.")
                    return
                else:
                    # 子目标到达后，清空当前规划，等待下次规划新段
              
                    self.global_trajps = None
                    self.current_subgoal = None
                    self.publish_state()
                    self.get_logger().info("Subgoal reached, planning next segment.")
                    return
                
            # 如果还没有规划轨迹，则进行规划
            if self.global_trajps is None:
                start = [0, 0]
                end = [target[0] - self.global_pose[0], target[1] - self.global_pose[1]]
                # 使用A*进行初步路径规划
                map_obj = GridMap()
                map_obj.init(grid_map_parms)
                map_obj.setMap(costmap)
                startp = np.array([[start[0]], [start[1]]])  # 转换为 2x1 的 ndarray
                endp = np.array([[end[0]], [end[1]]])  # 转换为 2x1 的 ndarray
                astar_res = map_obj.astarPlan(startp, endp, 0.5)
                astar_path = astar_res[0]

                # 利用轨迹优化器生成平滑轨迹
                traj_opt = PenalTrajOpt()
                traj_opt.init(grid_map_parms)
                traj_opt.setMap(costmap)
                if traj_opt.plan([start[0], start[1], self.global_pose[5]],
                                  [end[0], end[1], target[5]],
                                  0.5):
                    traj = traj_opt.getTraj()
                    traj_time = traj.getTotalDuration()
                    trajps = []
                    for i in range(100):
                        t = i / 100.0 * traj_time
                        pos = traj.getPos(t)
                        trajps.append((pos[0]+self.global_pose[0],pos[1]+self.global_pose[1],pos[2]))
                    trajps = np.array(trajps)
                    self.global_trajps = trajps
                    self.current_target = target  # 保存当前规划目标
                    self.state = "EXECUTING"
                    self.publish_state()
                    self.publish_path(self.global_trajps)
                else:
                    self.get_logger().warn("Trajectory optimization failed.")
                    return
            else:
                # 如果已有轨迹，直接进入执行状态
                self.state = "EXECUTING"
                self.publish_state()
                self.publish_path(self.global_trajps)

        elif self.state == "EXECUTING":

            # 检查是否到达当前规划目标
            if self.is_reached(target):
                if task_type == "global":
                    self.state = "COMPLETED"
                    self.publish_state()
                    self.get_logger().info("Global goal reached, task completed.")
                    return
                else:
                    # 子目标到达后，清空轨迹和子目标，下次重新规划
                    self.global_trajps = None
                    self.current_subgoal = None
                    self.state = "PLANNING"
                    self.publish_state()
                    self.get_logger().info("Subgoal reached, planning next segment.")
                    return
            else:
                # 保持之前规划的轨迹，持续发布
                self.publish_path(self.global_trajps)

        elif self.state == "COMPLETED":
            self.get_logger().info("Task completed. Waiting for new waypoint.")
            self.publish_state()
            return

def main(args=None):
    rclpy.init(args=args)
    node = GogoPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()