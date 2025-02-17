import numpy as np
from scipy.stats import entropy
import cupy as cp
from cupyx.scipy.ndimage import convolve as cp_convolve
from scipy.ndimage import gaussian_filter, median_filter, generic_filter
import time
from collections import deque

class TraversabilityAnalyzerWithBGK_GPU:
    def __init__(self, resolution=0.2, x_length=10, y_length=10, time_window=60.0, max_history_frames=5):
        """
        初始化类，设定地图的基本参数。

        Args:
            resolution (float): 栅格分辨率。
            x_length (float): 地图的x轴长度。
            y_length (float): 地图的y轴长度。
            time_window (float): 滑窗时间，单位为秒。
            max_history_frames (int): 最大历史帧数，限制保留的历史数据帧数。
        """
        self.resolution = resolution
        self.x_length = x_length
        self.y_length = y_length
        self.grid_size = int(x_length / resolution)
        self.map_size = self.grid_size**2

        # 全局滚动网格存储
        self.global_grid_offset = [0, 0]  # 初始偏移 (x_offset, y_offset)
        self.historical_observations = {}  # {cell: deque of (value, variance, count, timestamp)}
        self.time_window = time_window  # 滑窗时间（秒）
        self.max_history_frames = max_history_frames  # 最大历史帧数

    @staticmethod
    def normalize_attribute(attribute, global_min, global_max):
        """
        对输入属性进行全局归一化，将数据限制在 [0, 1] 范围内。

        Args:
            attribute (numpy.ndarray): 输入的属性值。
            global_min (float): 属性的全局最小值。
            global_max (float): 属性的全局最大值。

        Returns:
            numpy.ndarray: 归一化后的属性。
        """
        if global_max == global_min:
            return np.zeros_like(attribute)  # 避免归一化无意义
        attribute = np.asarray(attribute, dtype=np.float64)
        return np.clip((attribute - global_min) / (global_max - global_min + 1e-10), 0, 1)

    def preprocess_data(self, data, method="gaussian", **kwargs):
        """
        数据预处理，应用滤波和去噪。

        Args:
            data (numpy.ndarray): 输入数据。
            method (str): 滤波方法，可选 "gaussian", "median"。
            kwargs: 滤波参数。

        Returns:
            numpy.ndarray: 去噪后的数据。
        """
        if method == "gaussian":
            sigma = kwargs.get("sigma", 1.0)
            return gaussian_filter(data, sigma=sigma)
        elif method == "median":
            size = kwargs.get("size", 3)
            return median_filter(data, size=size)
        else:
            raise ValueError("Unsupported filtering method")

    def calculate_slope(self, grad_mean , max_slope ,min_slope ,test_slope):
        """
        使用梯度场计算坡度。

        Args:
            grad_mean (numpy.ndarray): 坐标梯度数组，形状为 (N, 2)。

        Returns:
            numpy.ndarray: 归一化后的坡度值，形状为 (N, )。
        """
        grad_mean = self.preprocess_data(grad_mean, method="gaussian", sigma=1.0)
        slope = np.linalg.norm(grad_mean, axis=1)
        if(test_slope):
            print(max(slope),min(slope))
        return self.normalize_attribute(slope, min_slope, max_slope)

    def calculate_flatness_entropy(self, height_map,max_flatness,min_flatness,test_flatness):
        p=self.preprocess_data(height_map, method="gaussian", sigma=0.5)
        if(test_flatness):
            print(max(p),min(p))
        return self.normalize_attribute(p, min_flatness, max_flatness)


    def calculate_step_height_topology(self, height_map,max_height,min_height,test_height):
        p=self.preprocess_data(height_map, method="gaussian", sigma=0.5)
        if(test_height):
            print(max(p),min(p))
        return self.normalize_attribute(p, min_height, max_height)

    def calculate_uncertainty_information_gain(self, preds, prior_entropy,max_uncertainty,min_uncertainty,test_uncertainty):
        """
        使用信息增益计算不确定性。

        Args:
            preds (numpy.ndarray): 每个点的预测分布标准差或其他不确定性度量。
            prior_entropy (float): 先验熵。

        Returns:
            numpy.ndarray: 信息增益后的不确定性。
        """
        preds = self.preprocess_data(preds, method="gaussian", sigma=0.5)
        posterior_entropy = -preds * np.log2(preds + 1e-10)  # 避免 log(0)
        info_gain = prior_entropy - posterior_entropy
        if(test_uncertainty):
            print(max(info_gain),min(info_gain))
        return self.normalize_attribute(info_gain, min_uncertainty, max_uncertainty)

    def update_global_grid_offset(self, new_position):
        """
        更新全局网格的偏移，以滚动窗口方式管理数据。

        Args:
            new_position (tuple): 当前机器人全局坐标 (x, y)。
        """
        x_offset = int(new_position[0] // self.resolution)
        y_offset = int(new_position[1] // self.resolution)

        delta_x = x_offset - self.global_grid_offset[0]
        delta_y = y_offset - self.global_grid_offset[1]

        if abs(delta_x) >= self.grid_size or abs(delta_y) >= self.grid_size:
            # 如果偏移超出网格大小，清空历史观测
            self.historical_observations.clear()
        else:
            # 移除超出边界的历史观测
            to_delete = [
                cell for cell in self.historical_observations
                if not (0 <= cell[0] - delta_x < self.grid_size and 0 <= cell[1] - delta_y < self.grid_size)
            ]
            for cell in to_delete:
                del self.historical_observations[cell]

        # 更新全局偏移
        self.global_grid_offset = [x_offset, y_offset]

    def clean_old_observations(self):
        """
        清理超出滑窗范围的历史观测数据。
        """
        current_time = time.time()
        for cell in list(self.historical_observations.keys()):
            # 保留时间窗口内的历史数据
            self.historical_observations[cell] = deque(
                [(value, variance, count, timestamp) for value, variance, count, timestamp in self.historical_observations[cell]
                 if current_time - timestamp <= self.time_window], maxlen=self.max_history_frames)
            
            if len(self.historical_observations[cell]) == 0:
                del self.historical_observations[cell]  # 清理空的条目

    def update_historical_observations(self, new_observations):
        """
        更新历史观测数据，将新观测与历史数据融合，并清理旧数据。

        Args:
            new_observations (dict): 新的观测数据，格式 {cell: (value, variance)}。
        """
        # 清理旧数据
        self.clean_old_observations()

        current_time = time.time()
        for cell, (new_value, new_variance) in new_observations.items():
            if cell in self.historical_observations:
                # 获取当前观测历史
                history = self.historical_observations[cell]
                
                # 只处理非空的历史数据
                if len(history) > 0:
                    # 初始化累计值和权重总和
                    total_value = 0
                    total_variance = 0
                    total_weight = 0

                    for old_value, old_variance, count, old_timestamp in history:
                        # 计算动态权重
                        time_diff = current_time - old_timestamp
                        time_weight = np.exp(-time_diff / self.time_window)  # 根据时间差的衰减权重
                        uncertainty_weight = 1 / (1 + old_variance)  # 根据不确定性调整权重
                        dynamic_weight = time_weight * uncertainty_weight

                        # 累加加权值
                        total_value += dynamic_weight * old_value
                        total_variance += dynamic_weight * old_variance
                        total_weight += dynamic_weight

                    # 添加新观测的加权值
                    total_value += new_value
                    total_variance += new_variance
                    total_weight += 1

                    # 更新历史数据，保存累计的加权值
                    updated_value = total_value / total_weight if total_weight > 0 else new_value
                    updated_variance = total_variance / total_weight if total_weight > 0 else new_variance

                    self.historical_observations[cell].append((updated_value, updated_variance, history[0][2] + 1, current_time))
            else:
                # 新栅格直接添加
                self.historical_observations[cell] = deque([(new_value, new_variance, 1, current_time)], maxlen=self.max_history_frames)


    def spatial_temporal_bgk_inference(self, grid_cells, kernel_function, variance_weight, radius=2.0):
        """
        使用 GPU 优化的时空 BGK 推断，结合历史数据。

        Args:
            grid_cells (list): 栅格坐标列表。
            kernel_function (callable): 核函数。
            variance_weight (float): 方差的权重。
            radius (float): 搜索半径，仅考虑邻域栅格。

        Returns:
            numpy.ndarray: 推断后的栅格值。
        """
        # 从历史观测中提取数据
        observed_coords = cp.array(list(self.historical_observations.keys()), dtype=cp.float32)

        # 检查历史观测数据是否为空
        if len(observed_coords) == 0:
            return cp.zeros(len(grid_cells), dtype=cp.float32).get()

        observed_values = []
        observed_variances = []

        for key in self.historical_observations:
            # 确保deque非空
            history = self.historical_observations[key]
            if len(history) > 0:
                observed_values.append(history[-1][0])  # 最新的观测值
                observed_variances.append(history[-1][1])  # 最新的方差

        # 转换为GPU数组
        observed_values = cp.array(observed_values, dtype=cp.float32)
        observed_variances = cp.array(observed_variances, dtype=cp.float32)

        grid_cells = cp.array(grid_cells, dtype=cp.float32)
        fused_values = cp.zeros(len(grid_cells), dtype=cp.float32)

        # 计算距离
        dists = cp.linalg.norm(grid_cells[:, None, :] - observed_coords[None, :, :], axis=2)

        # 筛选半径范围内的邻域
        mask = dists <= radius
        weights = kernel_function(dists * mask) / (observed_variances[None, :] + variance_weight)
        weights = cp.nan_to_num(weights, nan=0.0, posinf=1.0, neginf=0.0)
        weights_sum = weights.sum(axis=1, keepdims=True)

        # 避免除以 0
        weights_sum = cp.maximum(weights_sum, 1e-10)

        # 推断栅格值
        fused_values = (weights @ observed_values) / weights_sum.flatten()

        return fused_values.get()


    def calculate_traversability(self, slope, flatness, step_height, uncertainties, position,w_slope,w_flatness,w_step_height):
        """
        使用 BGK 推断结合历史数据计算可通行性。

        Args:
            slope (numpy.ndarray): 坡度值数组。
            flatness (numpy.ndarray): 平坦度值数组。
            step_height (numpy.ndarray): 台阶度值数组。
            uncertainties (numpy.ndarray): 不确定性数组。
            position (tuple): 当前机器人全局坐标 (x, y)。

        Returns:
            numpy.ndarray: 最终的可通行性评分，范围 [0, 1]。
        """
        self.update_global_grid_offset(position)

        map_size = int(cp.sqrt(len(slope)))
        grid_cells = [(i, j) for i in range(map_size) for j in range(map_size)]

        def kernel_function(distance, l=1.0):
            return cp.exp(-distance**2 / (2 * l**2))

        # 当前帧的观测数据
        traversability_pre = (slope) * w_slope+ (flatness) * w_flatness + (step_height) * w_step_height
        new_observations = {cell: (traversability_pre[idx], uncertainties[idx]) for idx, cell in enumerate(grid_cells)}

        # 更新历史观测并清理
        self.update_historical_observations(new_observations)

        # 使用历史观测进行时空推断
        traversability = traversability_pre#self.spatial_temporal_bgk_inference(grid_cells, kernel_function, variance_weight=0.5)

        min_num=np.min(traversability)
        max_num=np.max(traversability)
        
        # traversability=(traversability-min_num)/(max_num-min_num)
        
        range_val = max_num - min_num if max_num != min_num else 1e-10
        traversability = (traversability - min_num) / range_val
        
        # print(len(self.historical_observations))
        
        return traversability


# 示例代码
if __name__ == "__main__":
    analyzer = TraversabilityAnalyzerWithBGK_GPU()

    # 模拟输入数据
    grad_mean = cp.random.rand(2500, 2) * 30
    height_map = cp.random.rand(2500)
    uncertainties = cp.random.rand(2500) * 0.2
    height = cp.random.rand(2500)

    # 简化特性计算
    slope = cp.linalg.norm(grad_mean, axis=1)
    flatness = cp.abs(cp_convolve(height_map.reshape(50, 50), cp.ones((3, 3)) / 9).flatten())
    step_height = cp.abs(cp_convolve(height_map.reshape(50, 50), cp.array([[1, 0, -1], [1, 0, -1], [1, 0, -1]])).flatten())

    # 确保所有特征形状一致
    assert slope.shape == flatness.shape == step_height.shape == uncertainties.shape == height.shape, "Feature shapes are inconsistent!"

    # 使用优化算法计算可通行性
    traversability = analyzer.calculate_traversability(slope, flatness, step_height, uncertainties, height)

    print(f"Traversability: {traversability.get()}")