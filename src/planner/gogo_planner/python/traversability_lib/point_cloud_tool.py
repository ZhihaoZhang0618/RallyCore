import numpy as np
import open3d as o3d
# import numpy as np

import numpy as np

def pointcloud2_to_xyz(msg, z_threshold=2.0):
    """
    Convert a sensor_msgs/PointCloud2 message to a NumPy array of XYZ coordinates.
    Filters points based on a z-coordinate threshold.

    Parameters:
    - msg: The sensor_msgs/PointCloud2 message to convert.
    - z_threshold: A float specifying the maximum z-coordinate value to include in the output array.

    Returns:
    - A NumPy array of shape (N, 3) containing the filtered XYZ coordinates.
    """
    # Extract the fields and their offsets
    field_names = [field.name for field in msg.fields]
    field_offsets = [field.offset for field in msg.fields]

    # Find the indices of the x, y, z fields
    x_idx = field_names.index("x") if "x" in field_names else None
    y_idx = field_names.index("y") if "y" in field_names else None
    z_idx = field_names.index("z") if "z" in field_names else None

    if x_idx is None or y_idx is None or z_idx is None:
        raise ValueError("PointCloud2 message must contain x, y, and z fields.")

    # Convert the PointCloud2 data to a structured NumPy array
    dtype = np.dtype([(name, np.float32) for name in ["x", "y", "z"]])
    point_step = msg.point_step
    num_points = len(msg.data) // point_step

    # Ensure the buffer size is a multiple of the point step
    if len(msg.data) % point_step != 0:
        raise ValueError("PointCloud2 data size is not a multiple of point_step.")

    # Create a structured array from the buffer
    points = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)
    xyz = np.zeros((num_points, 3), dtype=np.float32)

    # Extract x, y, z values
    xyz[:, 0] = points[:, field_offsets[x_idx]:field_offsets[x_idx] + 4].view(np.float32).reshape(-1)
    xyz[:, 1] = points[:, field_offsets[y_idx]:field_offsets[y_idx] + 4].view(np.float32).reshape(-1)
    xyz[:, 2] = points[:, field_offsets[z_idx]:field_offsets[z_idx] + 4].view(np.float32).reshape(-1)

    # Filter points based on z-coordinate threshold
    mask = xyz[:, 2] < z_threshold
    filtered_xyz = xyz[mask]

    return filtered_xyz

def point_cloud2_to_array(msg, z_threshold):
    """
    Convert a sensor_msgs/PointCloud2 message to a NumPy array, filtering
    points based on a z-coordinate threshold. The fields in the PointCloud2
    message are mapped to the fields in the NumPy array as follows:
    * x, y, z -> X, Y, Z
    * intensity -> I (if present, otherwise filled with 1)
    * rgb and other fields are ignored

    Parameters:
    - msg: The sensor_msgs/PointCloud2 message to convert.
    - z_threshold: A float specifying the maximum z-coordinate value to include in the output array.

    Returns:
    - A NumPy array containing the filtered points with shape (N, 4), where the columns are [x, y, z, intensity].
    """
    # Get the index of the "intensity" field in the PointCloud2 message
    field_names = [field.name for field in msg.fields]
    intensity_idx = field_names.index("intensity") if "intensity" in field_names else None

    # Convert the PointCloud2 message to a NumPy array
    pc_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.zeros((pc_data.shape[0], 3), dtype=np.float32)

    # Extract x, y, z values
    xyz[:, 0] = pc_data[:, msg.fields[field_names.index("x")].offset:msg.fields[field_names.index("x")].offset + 4].view(np.float32).reshape(-1)
    xyz[:, 1] = pc_data[:, msg.fields[field_names.index("y")].offset:msg.fields[field_names.index("y")].offset + 4].view(np.float32).reshape(-1)
    xyz[:, 2] = pc_data[:, msg.fields[field_names.index("z")].offset:msg.fields[field_names.index("z")].offset + 4].view(np.float32).reshape(-1)

    # Filter points based on the z-coordinate threshold
    mask = xyz[:, 2] < z_threshold
    filtered_xyz = xyz[mask]

    # Add intensity if present, otherwise fill with 1
    if intensity_idx is not None:
        intensity = pc_data[mask, msg.fields[intensity_idx].offset:msg.fields[intensity_idx].offset + 4].view(np.float32).reshape(-1, 1)
    else:
        # If intensity is not present, fill with 1
        intensity = np.ones((filtered_xyz.shape[0], 1), dtype=np.float32)

    # Combine x, y, z, and intensity into a single array
    filtered_data = np.hstack((filtered_xyz, intensity))

    return filtered_data

def pointcloud2_to_array(cloud_msg,z_threshold):
    arr= point_cloud2_to_array(cloud_msg,z_threshold)   
    return arr

def voxel_downsample(pcl_arr, voxel_size):
    """
    使用体素滤波下采样点云数据，并保留强度信息
    """
    # 创建 Open3D 点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcl_arr[:, :3])  # 只取 x, y, z
    
    # 下采样
    downsampled_pcd = pcd.voxel_down_sample(voxel_size)
    downsampled_positions = np.asarray(downsampled_pcd.points)  # 下采样后的 x, y, z
    
    # 插值强度信息
    if pcl_arr.shape[1] == 4:  # 如果包含强度信息
        intensities = pcl_arr[:, 3]  # 提取强度
        # 使用最近邻插值
        from sklearn.neighbors import NearestNeighbors
        nbrs = NearestNeighbors(n_neighbors=1).fit(pcl_arr[:, :3])
        _, indices = nbrs.kneighbors(downsampled_positions)
        downsampled_intensities = intensities[indices].reshape(-1, 1)
        # 合并 x, y, z 和强度
        downsampled_pcl = np.hstack((downsampled_positions, downsampled_intensities))
    else:
        # 如果不包含强度信息，只返回 x, y, z
        downsampled_pcl = downsampled_positions
    
    return downsampled_pcl