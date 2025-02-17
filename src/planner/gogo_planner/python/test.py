import numpy as np
import open3d as o3d
from f_sgp_bgk import TraversabilityAnalyzer
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from planner import GridMap, indexToPos, PenalTrajOpt
import matplotlib as mpl

def generate_local_pointcloud(pose, num_points=1000, radius=2.5, noise_std=0.1, num_columns=10, column_radius=0.5, max_column_height=5):
    """
    生成围绕给定位姿的局部点云，其中包含几个柱子
    pose: [x, y, z, roll, pitch, yaw] - 机器人或传感器的位姿
    num_points: 点云中点的数量
    radius: 生成点云的半径
    noise_std: 每个点的噪声标准差，用于模拟高度变化
    num_columns: 柱子的数量
    column_radius: 柱子的半径
    max_column_height: 柱子的最大高度
    """
    # 生成围绕 pose 的点
    x, y, z, roll, pitch, yaw = pose
    angles = np.random.rand(num_points) * 2 * np.pi  # 角度
    distances = np.random.rand(num_points) * radius  # 半径

    # 生成 (x, y) 坐标
    x_points = x + distances * np.cos(angles)
    y_points = y + distances * np.sin(angles)

    # 初始化 z_points
    z_points = np.zeros(num_points)

    # 在随机位置生成柱子
    for _ in range(num_columns):
        # 随机选择柱子的位置
        column_center_x = x + np.random.uniform(-radius, radius)
        column_center_y = y + np.random.uniform(-radius, radius)
        # 随机选择柱子的高度
        column_height = np.random.uniform(0.5, max_column_height)
        
        # 计算柱子上的点
        distances_to_column = np.sqrt((x_points - column_center_x)**2 + (y_points - column_center_y)**2)
        
        # 如果距离柱子中心小于柱子的半径，则认为该点在柱子上
        column_points_mask = distances_to_column < column_radius
        
        # 将柱子上的点的 z 值设置为柱子的高度
        z_points[column_points_mask] = np.random.uniform(0.5, column_height, size=np.sum(column_points_mask))

    # 给定每个点的高度变化，用噪声来模拟
    z_points += np.random.randn(num_points) * noise_std  # 噪声控制高度变化

    # 组合为局部点云
    point_cloud = np.column_stack((x_points, y_points, z_points))

    # 清除 NaN 值
    point_cloud = point_cloud[~np.isnan(point_cloud).any(axis=1)]

    return point_cloud

def transform_to_local_coordinates(pose, point_cloud):
    """
    将全局坐标系下的点云转到局部坐标系
    pose: [x, y, z, roll, pitch, yaw] - 机器人或传感器的位姿
    point_cloud: 全局坐标系下的点云
    返回：局部坐标系下的点云
    """
    x, y, z, roll, pitch, yaw = pose

    # 构建旋转矩阵
    rotation = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    # 将点云从全局坐标系转换到局部坐标系
    transformed_points = np.dot(point_cloud - np.array([x, y, z]), rotation.T)

    return transformed_points




def color_point_cloud(pcd, values, colormap="jet"):
    """
    根据值给点云上色
    pcd: Open3D 点云对象
    values: 一个数组，包含每个点的值
    colormap: 颜色映射方式，可以选择不同的 colormap，如 "jet", "viridis" 等
    """
    values = np.array(values)
    values = (values - values.min()) / (values.max() - values.min())  # 正常化值
    
    # 归一化梯度值到 [0, 1]，反转顺序，越大越红
    values_normalized = 1 - values
    
    # 使用Matplotlib的colormap进行颜色映射
    cmap = plt.get_cmap('RdYlGn')  # 从红到绿
    values_colored = cmap(values_normalized)[:, :3]  # 获取 RGB 颜色，丢弃透明度通道
    
    # 确保 grad_mean_colored 是 float64 类型
    values_colored = values_colored.astype(np.float64)
    
    # 将 grad_mean_colored 展平为 (num_points, 3)
    pcd.colors = o3d.utility.Vector3dVector(values_colored.reshape(-1, 3))  # 使用颜色

    
    return pcd

def visualize_colored_point_cloud(pcd, title):
    """
    可视化上色后的点云
    pcd: Open3D 点云对象
    title: 可视化窗口标题
    """
    o3d.visualization.draw_geometries([pcd], window_name=title)
        

def generate_query_grid(pose, grid_size=10.0, resolution=0.2):
    """
    生成以当前pose为中心的查询网格
    pose: 机器人位姿 [x, y, z, roll, pitch, yaw]
    grid_size: 网格范围（以米为单位，5x5米）
    resolution: 网格分辨率（默认 0.2m）

    返回：
        - 查询网格点列表 (N, 2)  -> (x, y)
    """
    x_center, y_center = pose[:2]  # 提取位姿的 x, y 位置
    num_points_per_axis = int(grid_size / resolution)  # 计算每个轴上的点数
    
    x_range = np.linspace(x_center - grid_size / 2, x_center + grid_size / 2, num_points_per_axis)
    y_range = np.linspace(y_center - grid_size / 2, y_center + grid_size / 2, num_points_per_axis)

    # 生成网格点
    xx, yy = np.meshgrid(x_range, y_range)
    query_points = np.column_stack((xx.ravel(), yy.ravel()))  # 变成 (N, 2) 形状

    return query_points

def generate_costmap(query_points, traversability_values, grid_size_x, grid_size_y, resolution):
    """
    生成代价地图，并在边缘加上 1
    query_points: (N, 2) 查询点的 (x, y) 坐标
    traversability_values: (N,) 对应查询点的可通行性值
    grid_size_x/y: 地图尺寸
    resolution: 地图分辨率
    返回：
        - costmap (numpy 数组)
    """
    num_x = int(grid_size_x / resolution)
    num_y = int(grid_size_y / resolution)
    costmap = np.zeros((num_x, num_y))  # 先初始化为 1，保证四周是 1

    x_min, y_min = query_points[:, 0].min(), query_points[:, 1].min()
    for i, (x, y) in enumerate(query_points):
        idx_x = int((x - x_min) / resolution)
        idx_y = int((y - y_min) / resolution)
        
        if 0 < idx_x < num_x - 1 and 0 < idx_y < num_y - 1:  # 只填充内部区域
            if(traversability_values[i]>0.7):
                costmap[idx_x, idx_y] = 1

    return costmap

def main():
    # 生成一个随机全局位姿 (假设位姿是 [x, y, z, roll, pitch, yaw])
    pose = np.array([5.0, 3.0, 1.0, 0.0, 0.0, 0.0])  # 位置和姿态

    # 使用位姿生成全局点云
    pcl = generate_local_pointcloud(pose)

    # 将生成的点云转换到局部坐标系
    local_pcl = transform_to_local_coordinates(pose, pcl)

    # 假设有一个 TraversabilityAnalyzer 类
    analyzer = TraversabilityAnalyzer(config_path="../config/params.yaml")
    analyzer.update_map(pose, local_pcl)

    # 生成以 `pose` 为中心的 5x5 m 查询网格(全局坐标系下)
    query_points = generate_query_grid(pose)

    # 存储查询结果的高程和可通行性值
    traversability_values = []
    mean_xyz_values = []
    query_results = {}

    for x, y in query_points:
        result = analyzer.query(x, y)
        if result["traversability"] is None:
            traversability_values.append(0)  # 如果无数据，设为 0
        else:
            traversability_values.append(result["traversability"])
            
        if result["mean"] is None:
            mean_xyz_values.append(0)  # 如果无数据，设为 0
        else:
            mean_xyz_values.append([x,y,result["mean"]])
            
        query_results[(x, y)] = result

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mean_xyz_values)
    # 给点云上色
    colored_pcd = color_point_cloud(pcd, traversability_values, colormap="jet")

    # 可视化上色后的点云
    visualize_colored_point_cloud(colored_pcd, "Colored by Traversability")
    
    # 输出一些查询点的结果
    # print("Sample Query Results:")
    # for i, ((qx, qy), res) in enumerate(query_results.items()):
    #     if i > 10:  # 仅打印前 10 个查询点
    #         break
    #     print(f"Point ({qx:.2f}, {qy:.2f}) -> Traversability: {res['traversability']}")

    # 地图参数
    grid_map_parms = {
        "map_size_x": 10.0,
        "map_size_y": 10.0,
        "resolution": 0.2
    }
    
    costmap = generate_costmap(query_points, traversability_values, 10.0, 10.0, 0.2)
    # print(costmap)
    # **初始化 GridMap**
    map = GridMap()
    map.init(grid_map_parms)
    map.setMap(costmap)  # 使用可通行性代价地图
    sdf_map = map.getMap()

    # **绘制地图**
    x, y, z = [], [], []
    zi = []
    for i in range(sdf_map.shape[0]):
        for j in range(sdf_map.shape[1]):
            idx = np.array([[i], [j]], dtype=np.int32)
            pos = np.zeros((2, 1), dtype=np.float64)
            indexToPos(map, idx, pos)
            # print(pos[0],pos[1])
            x.append(pos[0])
            y.append(pos[1])
            z.append(sdf_map[i][j])
            zi.append(costmap[i][j])

    x = np.array(x).squeeze()
    y = np.array(y).squeeze()
    z = np.array(z).squeeze()
    zi = np.array(zi).squeeze()

    fig, viz_ax = plt.subplots(1, 2)

    levels = mpl.ticker.MaxNLocator(50).tick_values(z.min(), z.max())
    cp = viz_ax[0].tricontourf(x, y, z, levels)
    viz_ax[0].set_title("esdf map")
    viz_ax[0].set_xlabel("X")
    viz_ax[0].set_ylabel("Y")
    viz_ax[0].set_aspect('equal', 'box')
    fig.colorbar(cp, ax=viz_ax[0])

    cp = viz_ax[1].tricontourf(x, y, zi)
    viz_ax[1].set_title("grid map")
    viz_ax[1].set_xlabel("X")
    viz_ax[1].set_ylabel("Y")
    viz_ax[1].set_aspect('equal', 'box')
    fig.colorbar(cp, ax=viz_ax[1])

    # plot astar path
    astar_res = map.astarPlan([-4, -4], [4, 4], 0.5)
    astar_path = astar_res[0]
    astar_path = np.array(astar_path)
    viz_ax[1].plot(astar_path[:, 0], astar_path[:, 1], 'ro')

    # test planner
    traj_opt = PenalTrajOpt()
    traj_opt.init(grid_map_parms)
    traj_opt.setMap(costmap)
    if traj_opt.plan([-4, -4, 1.57], [4, 4, 0.0], 0.5):
        traj = traj_opt.getTraj()
        traj_time = traj.getTotalDuration()
        trajps = []
        for i in range(100):
            t = i / 100.0 * traj_time
            pos = traj.getPos(t)
            trajps.append(pos)
        trajps = np.array(trajps)
        viz_ax[1].plot(trajps[:, 0], trajps[:, 1], 'g-')
    plt.show()

if __name__ == "__main__":
    main()