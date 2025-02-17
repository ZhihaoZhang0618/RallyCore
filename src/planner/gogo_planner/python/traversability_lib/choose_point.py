import numpy as np
import torch
import open3d as o3d
from sklearn.neighbors import NearestNeighbors

def gpu_knn_search(pcl_tensor, k=20):
    """
    使用 GPU 进行 k 近邻搜索
    Parameters:
        pcl_tensor: (N, 3) 的点云张量
        k: 近邻点的数量
    Returns:
        distances: 每个点到其近邻点的距离 (N, k)
        indices: 每个点的近邻点索引 (N, k)
    """
    # 使用 torch.cdist 计算距离矩阵
    distances = torch.cdist(pcl_tensor, pcl_tensor)
    # 取最小的 k 个距离及其索引
    distances, indices = torch.topk(distances, k=k, largest=False)
    return distances, indices

def calculate_curvatures_gpu(pcl_tensor, indices):
    """
    使用 GPU 批量计算点云曲率
    Parameters:
        pcl_tensor: (N, 3) 的点云位置张量
        indices: 每个点的邻域索引 (N, k)
    Returns:
        curvatures: (N,) 每个点的曲率
    """
    neighbors = pcl_tensor[indices]  # (N, k, 3)
    mean = neighbors.mean(dim=1, keepdim=True)  # 计算邻域中心
    centered = neighbors - mean
    covariance = torch.einsum('bij,bik->bjk', centered, centered) / (neighbors.shape[1] - 1)
    eigenvalues = torch.linalg.eigvalsh(covariance)  # (N, 3)
    curvatures = eigenvalues[:, 0] / (eigenvalues.sum(dim=1) + 1e-6)
    return curvatures

def calculate_gradients_gpu(pcl_tensor, indices):
    """
    使用 GPU 批量计算点云梯度
    Parameters:
        pcl_tensor: (N, 3) 的点云位置张量
        indices: 每个点的邻域索引 (N, k)
    Returns:
        gradients: (N,) 每个点的梯度
    """
    neighbors = pcl_tensor[indices]  # (N, k, 3)
    dz = neighbors[:, :, 2] - pcl_tensor[:, 2].unsqueeze(1)  # 计算 z 方向梯度
    gradients = torch.mean(torch.abs(dz), dim=1)
    return gradients

import numpy as np

def extract_features_with_classification_gpu(pcl_arr, curvature_threshold=0.1, gradient_threshold=0.05, voxel_size=0.2, target_num_points=10000):
    """
    使用 GPU 加速的特征提取算法，支持 (N, 4) 输入
    """
    # 检查输入维度
    assert pcl_arr.shape[1] == 4, "输入点云数据必须为 (N, 4)，包括强度信息。"

    # 提取点云位置和强度
    pcl_positions = pcl_arr[:, :3]  # 提取位置 (x, y, z)
    pcl_intensities = pcl_arr[:, 3]  # 提取强度

    # 将位置数据转为 GPU 张量
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    pcl_tensor = torch.tensor(pcl_positions, dtype=torch.float32).to(device)

    # 使用 GPU 进行 k 近邻搜索
    _, indices = gpu_knn_search(pcl_tensor, k=20)

    # 批量计算曲率和梯度
    curvatures = calculate_curvatures_gpu(pcl_tensor, indices)
    gradients = calculate_gradients_gpu(pcl_tensor, indices)

    # 判断特征明显区域
    is_feature = (curvatures > curvature_threshold) | (gradients > gradient_threshold)
    feature_points = pcl_arr[is_feature.cpu().numpy()]  # 保留特征点 (N, 4)

    # 添加特征值列（曲率或梯度）到特征点
    feature_curvatures = curvatures[is_feature.cpu().numpy()].cpu().numpy()
    feature_gradients = gradients[is_feature.cpu().numpy()].cpu().numpy()

    feature_points_with_values = np.hstack((feature_points, feature_curvatures.reshape(-1, 1), feature_gradients.reshape(-1, 1)))

    # 特征不明显区域：均匀下采样
    non_feature_points = pcl_arr[~is_feature.cpu().numpy()]  # (N, 4)
    if len(non_feature_points) > 0:
        non_feature_positions = non_feature_points[:, :3]
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(non_feature_positions)
        downsampled_pcl = point_cloud.voxel_down_sample(voxel_size)
        downsampled_positions = np.asarray(downsampled_pcl.points)

        # 插值强度信息
        downsampled_indices = NearestNeighbors(n_neighbors=1).fit(non_feature_positions).kneighbors(downsampled_positions, return_distance=False)
        average_intensities = np.mean(pcl_intensities[downsampled_indices], axis=1)
        downsampled_points = np.hstack((downsampled_positions, average_intensities.reshape(-1, 1)))

        # 扩展 downsampled_points 为 (M, 6)，添加曲率和梯度列
        downsampled_points_extended = np.hstack((downsampled_points, np.zeros((downsampled_points.shape[0], 2))))  # 填充曲率和梯度为零
    else:
        downsampled_points_extended = np.empty((0, 6))  # 如果没有非特征点，则返回一个空数组

    # 合并结果
    processed_pcl = np.vstack((feature_points_with_values, downsampled_points_extended))

    # 如果总点数超过目标数量，随机采样限制数量
    if len(processed_pcl) > target_num_points:
        indices = np.random.choice(len(processed_pcl), target_num_points, replace=False)
        processed_pcl = processed_pcl[indices]

    return processed_pcl