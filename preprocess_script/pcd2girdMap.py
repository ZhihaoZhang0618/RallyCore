import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
import os
import glob
import argparse
from PIL import Image

# 全局常量
RESOLUTION = 0.05  # 地图分辨率（米/像素）
POINT_SIZE = 2     # 每个点的大小（像素）

def get_latest_map_folder(base_path="~/maps"):
    """获取最新的地图文件夹"""
    base_path = os.path.expanduser(base_path)
    folders = glob.glob(os.path.join(base_path, "*"))
    folders = [f for f in folders if os.path.isdir(f)]
    return max(folders, key=os.path.getmtime) if folders else None

def get_user_input_path(prompt, default_path):
    """获取用户输入的路径"""
    print(f"\n{prompt}")
    print(f"默认路径: {default_path}")
    print("按Enter使用默认路径，或输入新路径:")
    user_input = input().strip()
    if not user_input:
        return default_path
    return user_input if os.path.exists(user_input) else default_path

def get_user_input_range(min_val, max_val, axis_name):
    """获取用户输入的范围"""
    print(f"\n{axis_name}轴范围: {min_val:.2f}到{max_val:.2f}米")
    print("输入裁剪范围(格式: 最小值,最大值)，按Enter使用完整范围:")
    user_input = input().strip()
    if not user_input:
        return [min_val, max_val]
    try:
        min_input, max_input = map(float, user_input.split(','))
        min_input = max(min_input, min_val)
        max_input = min(max_input, max_val)
        if min_input >= max_input:
            print("输入有误，使用完整范围")
            return [min_val, max_val]
        return [min_input, max_input]
    except:
        print("输入格式错误，使用完整范围")
        return [min_val, max_val]

def filter_point_cloud(points, x_range, y_range, z_range):
    """根据XYZ范围过滤点云"""
    mask = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) & \
           (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) & \
           (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    return points[mask]

def visualize_point_cloud(points, filtered_points):
    """可视化原始和过滤后的点云"""
    pcd_original = o3d.geometry.PointCloud()
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_original.points = o3d.utility.Vector3dVector(points)
    pcd_filtered.points = o3d.utility.Vector3dVector(filtered_points)
    pcd_original.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_filtered.paint_uniform_color([1, 0, 0])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="点云可视化", width=1280, height=720)
    vis.add_geometry(pcd_original)
    vis.add_geometry(pcd_filtered)
    vis.add_geometry(frame)
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -1])
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, -1, 0])
    vis.get_render_option().point_size = 2.0
    vis.get_render_option().background_color = [0, 0, 0]
    vis.run()
    vis.destroy_window()

def pcd_to_gridmap(points, x_range, y_range, z_range, show_visualization=False):
    """将点云数据转换为2D网格地图"""
    filtered_points = filter_point_cloud(points, x_range, y_range, z_range)
    if show_visualization:
        visualize_point_cloud(points, filtered_points)
    width = int(np.ceil((x_range[1] - x_range[0]) / RESOLUTION))
    height = int(np.ceil((y_range[1] - y_range[0]) / RESOLUTION))
    grid_map = np.ones((width, height), dtype=np.uint8) * 255
    for p in filtered_points:
        gx = int((p[0] - x_range[0]) / RESOLUTION)
        gy = int((p[1] - y_range[0]) / RESOLUTION)
        if 0 <= gx < width and 0 <= gy < height:
            half = POINT_SIZE // 2
            for i in range(-half, half+1):
                for j in range(-half, half+1):
                    nx, ny = gx+i, gy+j
                    if 0 <= nx < width and 0 <= ny < height:
                        grid_map[nx, ny] = 0
    grid_map = binary_dilation(grid_map == 0, iterations=1).astype(np.uint8) * 255
    grid_map = 255 - grid_map
    origin = np.array([x_range[0], y_range[0], 0.0])
    print(f"地图尺寸: {width}x{height}像素, 实际大小: {width*RESOLUTION:.2f}x{height*RESOLUTION:.2f}米")
    return grid_map, origin

def save_gridmap(grid_map, origin, output_path):
    """保存网格地图为图像文件及YAML（含逆时针旋转90°）"""
    # 原始尺寸
    width, height = grid_map.shape
    # 逆时针旋转90°
    rotated = np.rot90(grid_map, k=1)
    # 保存PNG
    img = Image.fromarray(rotated)
    img.save(output_path)
    # 保存预览
    preview = output_path.replace('.png', '_preview.png')
    plt.imsave(preview, rotated, cmap='gray', origin='lower')
    # 重新计算origin: 旋转后像素(0,0)对应原(grid_x=0, grid_y=width-1)
    x0 = origin[0]
    y0 = origin[1] 
    rotated_origin = np.array([x0, y0, origin[2]])
    # 写YAML
    yaml_path = output_path.replace('.png', '.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f"image: {os.path.basename(output_path)}\n")
        f.write(f"resolution: {RESOLUTION}\n")
        f.write(f"origin: [{rotated_origin[0]}, {rotated_origin[1]}, {rotated_origin[2]}]\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("negate: 0\n")
    print(f"YAML文件保存到: {yaml_path}")
    print(f"旋转后原点: [{rotated_origin[0]:.2f}, {rotated_origin[1]:.2f}, {rotated_origin[2]:.2f}]")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='将PCD点云转换为旋转栅格地图')
    parser.add_argument('--visualize', '-v', action='store_true', help='显示点云可视化')
    args = parser.parse_args()
    latest = get_latest_map_folder()
    if latest is None:
        print("未找到地图文件夹，请确保~/maps目录存在")
        exit(1)
    default_pcd = os.path.join(latest, "map.pcd")
    default_out = os.path.join(latest, "map.png")
    pcd_path = get_user_input_path("输入PCD文件路径:", default_pcd)
    out_path = get_user_input_path("输入输出文件路径:", default_out)
    print("读取点云...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)
    x_min, y_min, z_min = np.min(pts, axis=0)
    x_max, y_max, z_max = np.max(pts, axis=0)
    z_rng = get_user_input_range(z_min, z_max, 'Z')
    x_rng = get_user_input_range(x_min, x_max, 'X')
    y_rng = get_user_input_range(y_min, y_max, 'Y')
    print("生成栅格地图...")
    gm, ori = pcd_to_gridmap(pts, x_rng, y_rng, z_rng, show_visualization=args.visualize)
    print("保存地图...")
    save_gridmap(gm, ori, out_path)
