import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
import traversability_lib.choose_point as choose_point
from sklearn.decomposition import PCA
from sklearn.feature_selection import mutual_info_regression
import torch
import gpytorch
from traversability_lib.sgp_model import SGPModel
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from traversability_lib.traversability import TraversabilityAnalyzerWithBGK_GPU
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
import yaml
from traversability_lib import point_cloud_tool

class TraversabilityAnalyzer:
    def __init__(self, config_path="../config/params.yaml"):
        # 读取 YAML 配置
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)

        # 参数加载
        self.curvature_threshold = config["curvature_threshold"]
        self.gradient_threshold = config["gradient_threshold"]
        self.key_voxel_size = config["key_voxel_size"]
        self.inducing_points = config["inducing_points"]
        self.lengthscale = config["lengthscale"]
        self.alpha = config["alpha"]

        self.resolution = config["resolution"]
        self.x_length = config["x_length"]
        self.y_length = config["y_length"]

        self.max_slope = config["max_slope"]
        self.min_slope = config["min_slope"]
        self.test_slope = config["test_slope"]

        self.max_flatness = config["max_flatness"]
        self.min_flatness = config["min_flatness"]
        self.test_flatness = config["test_flatness"]

        self.max_height = config["max_height"]
        self.min_height = config["min_height"]
        self.test_height = config["test_height"]

        self.max_uncertainty = config["max_uncertainty"]
        self.min_uncertainty = config["min_uncertainty"]
        self.test_uncertainty = config["test_uncertainty"]

        self.w_slope = config["w_slope"]
        self.w_flatness = config["w_flatness"]
        self.w_step_height = config["w_step_height"]

        # 设备选择
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # PCA 组件
        self.pca = PCA(n_components=4)

        # 地图相关
        self.grid = None
        self.Xs = None
        self.Ys = None
        self.Zs = None
        self.curvatures = None
        self.gradients = None

        # 分析器初始化
        self.analyzer = TraversabilityAnalyzerWithBGK_GPU()
        
        self.mean=None
        self.grad_mean=None
        self.slope = None
        self.flatness = None
        self.step_height = None
        self.uncertainty = None
        self.traversability = None
        
        self.pose=None
        self.traversability_dict = None
        self.kd_tree = None
        self.data_dict = None

    def knn_interpolate(self,pcl, num_points):
        """
        使用 K-近邻插值方法增加点云的数量，并添加强度值（第四列）。
        """
        num_original_points = pcl.shape[0]
        if num_original_points >= num_points:
            print("原始点云已经足够多，无需增加。")
            return np.hstack([pcl, np.ones((num_original_points, 1))])  # 添加强度列

        # 计算需要增加的点数
        points_to_add = num_points - num_original_points

        # 使用 KNN 寻找邻近的点
        knn = NearestNeighbors(n_neighbors=2)  # 只考虑最邻近的两个点
        knn.fit(pcl)
        distances, indices = knn.kneighbors(pcl)

        # 计算每对邻近点之间插值的数量
        points_per_pair = max(1, points_to_add // (num_original_points - 1))

        # 对每个点，根据它的邻居生成多个新点
        interpolated_points = []
        for i in range(num_original_points):
            # 获取最邻近的两个点
            neighbor_indices = indices[i]
            p1 = pcl[neighbor_indices[0]]
            p2 = pcl[neighbor_indices[1]]

            # 生成多个新点
            for j in range(points_per_pair):
                lambd = (j + 1) / (points_per_pair + 1)
                new_point = p1 + lambd * (p2 - p1)
                interpolated_points.append(new_point)

        # 将原始点云和插值后的点合并
        interpolated_points = np.array(interpolated_points)
        new_pcl = np.vstack([pcl, interpolated_points])

        # 确保最终点数匹配 num_points
        new_pcl = new_pcl[:num_points]

        # 添加强度列（第四列全为1）
        intensity_column = np.ones((new_pcl.shape[0], 1))
        new_pcl_with_intensity = np.hstack([new_pcl, intensity_column])

        return new_pcl_with_intensity

    def visualize_point_cloud_with_keypoints(self,pcl, keypoints):
        """
        使用 Open3D 可视化点云，同时用球体显示特征点。
        """
        if pcl.shape[1] == 4:  # 去掉强度列，仅显示XYZ
            pcl = pcl[:, :3]
        
        # 原始点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl)

        # 颜色为白色
        pcd.colors = o3d.utility.Vector3dVector(np.full((pcl.shape[0], 3), 0.8))

        # 特征点用球体表示
        keypoint_spheres = []
        for keypoint in keypoints:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)  # 小球半径
            sphere.translate(keypoint[:3])  # 移动到特征点位置
            sphere.paint_uniform_color([1, 0, 0])  # 颜色设为红色
            keypoint_spheres.append(sphere)

        # 显示点云和特征点球体
        o3d.visualization.draw_geometries([pcd] + keypoint_spheres)

    def check_independence(self,data, correlation_threshold=0.1, mi_threshold=0.1):
            """
            检查输入变量是否独立。
            
            参数:
                data (numpy.ndarray): 输入数据，形状为 (N, 4)，每一列分别为 x, y, Curvature, Gradient。
                correlation_threshold (float): 相关系数的阈值，默认值为 0.1。
                mi_threshold (float): 互信息的阈值，默认值为 0.1。
            
            返回:
                bool: 如果变量独立，返回 True；否则返回 False。
            """
            # 计算相关系数矩阵
            corr_matrix = np.corrcoef(data, rowvar=False)
            print("相关系数矩阵:")
            print(str(corr_matrix))
            
            # 检查相关系数是否低于阈值
            corr_independent = np.all(np.abs(corr_matrix - np.diag(np.diag(corr_matrix))) < correlation_threshold)
            
            # 计算互信息矩阵
            n_features = data.shape[1]
            mi_matrix = np.zeros((n_features, n_features))
            
            for i in range(n_features):
                for j in range(n_features):
                    if i != j:
                        mi_matrix[i, j] = mutual_info_regression(data[:, i].reshape(-1, 1), data[:, j])
            
            print("互信息矩阵:")
            print(str(mi_matrix))
            
            # 检查互信息是否低于阈值
            mi_independent = np.all(mi_matrix - np.diag(np.diag(mi_matrix)) < mi_threshold)
            
            # 综合判断
            if corr_independent and mi_independent:
                print("变量独立。")
                return True
            else:
                print("变量不独立。")
                return False
        
        
    def sampling_grid(self):
        """
        生成采样网格，用于生成均匀的点云，并补全缺失的曲率和梯度
        """
        # global grid,Xs,Ys,Zs,curvatures,gradients
        x_range = self.x_length / 2
        y_range = self.y_length / 2
        x_s = np.arange(-x_range, x_range, self.resolution, dtype='float32')
        y_s = np.arange(-y_range, y_range, self.resolution, dtype='float32')

        # 生成网格
        grid = np.array(np.meshgrid(x_s, y_s)).T.reshape(-1, 2)

        # 获取训练集数据
        X_train = np.column_stack((self.Xs, self.Ys))  # 训练集坐标
        curvatures_train = self.curvatures  # 训练集曲率
        gradients_train = self.gradients  # 训练集梯度

        # print(curvatures_train,gradients_train)
        # 使用 KNN 算法找到最近邻
        knn = NearestNeighbors(n_neighbors=5, algorithm='auto').fit(X_train)
        distances, indices = knn.kneighbors(grid)

        # 计算加权平均 (反距离加权)
        weights = 1 / (distances + 1e-6)  # 避免除以零
        weights /= np.sum(weights, axis=1, keepdims=True)  # 归一化权重

        # 插值曲率和梯度
        curvatures_pred = np.sum(weights[:, :, None] * curvatures_train[indices], axis=1)
        gradients_pred = np.sum(weights[:, :, None] * gradients_train[indices], axis=1)

        # 将补全的曲率和梯度添加到网格数据中
        self.grid = np.column_stack((grid, curvatures_pred, gradients_pred))

    def visualize_mean_grad(self,grid, mean, grad_mean):
        """
        使用 Open3D 可视化 mean（作为点的位置）和 grad_mean（作为点的颜色）
        grid: 2D 网格点 (X, Y)
        mean: 预测的高程值 (Z)
        grad_mean: 梯度信息，用作颜色
        """
        # 组合 X, Y, Z 形成 3D 点云
        points = np.hstack((grid[:, :2], mean.reshape(-1, 1)))
        
        # 归一化梯度值到 [0, 1]，反转顺序，越大越红
        grad_mean_normalized = 1 - np.clip((grad_mean - np.min(grad_mean)) / (np.max(grad_mean) - np.min(grad_mean)), 0, 1)
        
        # 使用Matplotlib的colormap进行颜色映射
        cmap = plt.get_cmap('RdYlGn')  # 从红到绿
        grad_mean_colored = cmap(grad_mean_normalized)[:, :3]  # 获取 RGB 颜色，丢弃透明度通道
        
        # 确保 grad_mean_colored 是 float64 类型
        grad_mean_colored = grad_mean_colored.astype(np.float64)
        
        # 构造 Open3D 点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 将 grad_mean_colored 展平为 (num_points, 3)
        pcd.colors = o3d.utility.Vector3dVector(grad_mean_colored.reshape(-1, 3))  # 使用颜色

        # 可视化
        o3d.visualization.draw_geometries([pcd])

    def filter_low_uncertainty_data(self,mean, var,grad_mean,grid,threshold=1.35):
        """
        过滤掉不确定性高的区域，只保留不确定性低于阈值的点。

        参数:
        - mean: 预测的均值 (Z)
        - var: 方差 (不确定性)
        - uncertainty: 不确定性信息
        - threshold: 不确定性阈值，默认0.8，越小保留的数据越多

        返回:
        - filtered_mean: 过滤后的均值
        - filtered_var: 过滤后的方差
        - filtered_uncertainty: 过滤后的不确定性
        """
        # 筛选出不确定性低于阈值的区域
        low_uncertainty_indices = np.where(var < threshold)[0]

        # 根据筛选的索引过滤数据
        filtered_mean = mean[low_uncertainty_indices]
        filtered_var = var[low_uncertainty_indices]
        filtered_grad_mean=grad_mean[low_uncertainty_indices]
        filtered_grid=grid[low_uncertainty_indices]
        
        return filtered_mean,filtered_var,filtered_grad_mean,filtered_grid

    def generate_robot_points(self, l, w):
        """
        生成机器人本体附近的假设点。
        假设机器人本体附近的区域是平坦的（高度为0），曲率和
        梯度也为0。
        """
        # 生成机器人本体附近的点
        # l=0.5
        # w=0.5
        self.i_num=15
        self.height=-0.1
        x = np.linspace(-l/2, l/2, num=self.i_num)
        y = np.linspace(-w/2, w/2, num=self.i_num)
        x, y = np.meshgrid(x, y)
        x = x.flatten()
        y = y.flatten()
        z = np.full_like(x, self.height)  # 将 z 扩展为与 x 和 y 相同的形状
        i= np.full_like(x, 0)
        c= np.full_like(x, 0)
        g= np.full_like(x, 0)
        # 组合成点云数据
        robot_points = np.column_stack((x, y, z ,i, c , g))
        
        return robot_points

    def generate_local_traversability_map(self,pose, transformed_points):
        """
        使用给定的位姿 (pose) 和变换后的点云 (transformed_points) 生成局部可通行性地图
        """
        self.pose=pose
        # global grid,Xs,Ys,Zs,curvatures,gradients
        # 增加点云数量到 2000，并添加强度列
        expanded_pcl = self.knn_interpolate(transformed_points, 2000)
        expanded_pcl=point_cloud_tool.voxel_downsample(expanded_pcl, voxel_size=0.2)
        print(f"提取前点云数量: {expanded_pcl.shape[0]}")
        # return
        # 传入增强后的点云进行特征点提取
        key_points = choose_point.extract_features_with_classification_gpu(
            expanded_pcl,  # 只传递 XYZ
            curvature_threshold=self.curvature_threshold,
            gradient_threshold=self.gradient_threshold,
            voxel_size=self.key_voxel_size,
            target_num_points=self.inducing_points
        )
        print(f"提取的特征点数量: {key_points.shape[0]}")
        
        l = self.x_length  # 机器人长度
        w = self.y_length  # 机器人宽度
        
        # 生成诱导点基座
        robot_points = self.generate_robot_points(l, w)
        key_points = np.vstack((key_points, robot_points))
        # return
        # 在 `key_points` 中已经包含了 (X, Y, Z, curvature, gradient)
        self.Xs = key_points[:, 0].reshape(-1, 1)  # X 坐标
        self.Ys = key_points[:, 1].reshape(-1, 1)  # Y 坐标
        self.Zs = key_points[:, 2].reshape(-1, 1)  # Z 坐标
        self.curvatures = key_points[:, 4].reshape(-1, 1)  # 曲率
        self.gradients = key_points[:, 5].reshape(-1, 1)  # 梯度
        
        # self.visualize_point_cloud_with_keypoints(expanded_pcl, key_points)

        # 检查变量独立性（仅检查一次）
        # if not self.independence_checked:
        data = np.column_stack((self.Xs,self.Ys,self.curvatures,self.gradients))
        # 对输入数据进行 PCA 降维
        data_pca = self.pca.fit_transform(data)
        # check_independence(data_pca)
        

        # 准备训练数据：输入是坐标(X, Y) + 强度 + 曲率 + 梯度，输出是高度(Z)
        d_in = torch.tensor(data_pca, dtype=torch.float32, device=self.device)
        d_out = torch.tensor(self.Zs, dtype=torch.float32, device=self.device).squeeze()
        # 移动数据张量到正确的设备
        data_in_tensor = torch.tensor(d_in, dtype=torch.float32, device=self.device)
        data_out_tensor = torch.tensor(d_out, dtype=torch.float32, device=self.device).squeeze()

        # 创建高斯过程模型并将其移动到正确的设备
        likelihood = gpytorch.likelihoods.GaussianLikelihood(mean=0).to(self.device)
        sgp_model = SGPModel(data_in_tensor, data_out_tensor, likelihood, self.inducing_points,self.lengthscale, self.alpha).to(self.device)

        # 训练高斯过程模型
        sgp_model.train()
        likelihood.train()
        optimizer = torch.optim.Adam(sgp_model.parameters(), lr=0.01)
        mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, sgp_model)

        optimizer.zero_grad()
        output = sgp_model(data_in_tensor)
        loss = -mll(output, data_out_tensor).mean()
        loss.backward()
        optimizer.step()

        #构建测试集
        self.sampling_grid()
        # return
        # print(grid)
        # return
        # 获取预测的均值和方差
        Xtest_tensor = torch.tensor(self.pca.transform(self.grid), dtype=torch.float32, requires_grad=True).to(sgp_model.device)
        sgp_model.eval()
        likelihood.eval()
        preds = sgp_model.likelihood(sgp_model(Xtest_tensor))
        mean = preds.mean.detach().cpu().numpy()
        var = preds.variance.detach().cpu().numpy()
        # return

        # 计算梯度（grad_mean）
        grad_mean = torch.autograd.grad(preds.mean.sum(), Xtest_tensor, create_graph=True)[0].detach().cpu().numpy()

        filtered_mean,filtered_var,filtered_grad_mean,filter_grid=self.filter_low_uncertainty_data(mean,var,grad_mean,self.grid)
        # print(filtered_mean.shape,filtered_var.shape,filtered_grad_mean.shape,filter_grid.shape)
        self.mean=mean
        self.grad_mean=filtered_grad_mean
        self.slope = self.analyzer.calculate_slope(filtered_grad_mean,self.max_slope,self.min_slope,self.test_slope)
        self.flatness = self.analyzer.calculate_flatness_entropy(np.array(filter_grid[:, 2]), self.max_flatness,self.min_flatness,self.test_flatness)
        self.step_height = self.analyzer.calculate_step_height_topology(np.array(filter_grid[:, 3]),self.max_height,self.min_height,self.test_height)
        self.uncertainty = self.analyzer.calculate_uncertainty_information_gain(filtered_var, 1,self.max_uncertainty,self.min_uncertainty,self.test_uncertainty)
        current_position=(pose[0],pose[1])
        self.traversability = 1.0-self.analyzer.calculate_traversability(self.slope, self.flatness, self.step_height, self.uncertainty, current_position,self.w_slope,self.w_flatness,self.w_step_height)
        # print(mean.shape)
        # print(slope.shape)
    
        # self.visualize_mean_grad(filter_grid,filtered_mean,self.traversability)

        # 可视化点云和特征点
        # self.visualize_point_cloud_with_keypoints(expanded_pcl, key_points)
        # 保存坐标与可通行性值的键值对
        self.traversability_dict={}
        for i in range(len(filter_grid)):
            # Ensure that the index i is within the bounds of traversability
            if i < len(self.traversability):
                coord = (filter_grid[i][0], filter_grid[i][1], filtered_mean[i])  # Convert list to tuple
                self.traversability_dict[coord] = (self.traversability[i],self.slope[i],self.flatness[i],self.step_height[i],self.uncertainty[i],self.grad_mean[i])

        self.global_traversability_list = []
         # 遍历局部可通行性图，将每个点的坐标从局部坐标系转换到全局坐标系
        for local_coord, traversability_value in self.traversability_dict.items():
            local_coord = np.array(local_coord)
            global_coord = self.convert_odom_to_se3(local_coord)
            x, y = global_coord[:2]  # 取 x, y 坐标
            z = global_coord[2] if len(global_coord) > 2 else 0  # 若没有 z，则设为 0
            self.global_traversability_list.append(
                (x, 
                 y, 
                 z, 
                 traversability_value[0],
                 traversability_value[1],
                 traversability_value[2],
                 traversability_value[3],
                 traversability_value[4],
                 traversability_value[5]))
            
        # 构建 KDTree 和查询字典
        self.kd_tree = KDTree([(x, y) for x, y, _, _,_ ,_ ,_ ,_,_ in self.global_traversability_list])
        self.data_dict = {(x, y): (z, traversability,slope,flatness,step_height,uncertainty,grad_mean) 
                          for x, y, z, traversability,slope,flatness,step_height,uncertainty,grad_mean in self.global_traversability_list}

        # # print(traversability_dict)
        # return traversability_dict

    def update_map(self, pose, local_pointcloud):
        #pose: [x, y, z, roll, pitch, yaw] - 机器人或传感器的位姿
        #local_pointcloud: 局部坐标系下的点云
        local_pointcloud=np.array(local_pointcloud)
        self.generate_local_traversability_map(pose, local_pointcloud)
        
    def globalxy2local(self,global_x,global_y):
        
        x, y, z, roll, pitch, yaw = self.pose

        point_cloud=np.array([global_x, global_y, z])
        
        # 构建旋转矩阵
        rotation = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

        # 将点云从全局坐标系转换到局部坐标系
        transformed_points = np.dot(point_cloud - np.array([x, y, z]), rotation.T)
        local_x=transformed_points[0]
        local_y=transformed_points[1]
        return local_x,local_y
    
    
    def convert_odom_to_se3(self, point):
        x, y, z, roll, pitch, yaw = self.pose
        position = (x,y,z)  # 位置(x, y, z)
        rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()  # 旋转矩阵
        rotation_matrix = rotation_matrix
        # 反向转换：全局坐标 -> 局部坐标
        # 先进行旋转反变换，可能不需要取转置，视具体坐标系定义而定
        rotated_point = np.dot(rotation_matrix, point)  # 使用旋转矩阵直接进行旋转
        # 然后进行平移反变换
        local_point = rotated_point + position  # 使用全局位置反向平移
        
        return local_point
    
    def query(self, x, y, max_radius=2.0):
        """
        查询某个全局位置的可通行性信息
        """
        dist, idx = self.kd_tree.query((x, y))  # 找最近邻
        nearest_x, nearest_y = self.kd_tree.data[idx]
        mean, traversability, slope, flatness, step_height, uncertainty, grad_mean = self.data_dict[(nearest_x, nearest_y)]
        
        if dist > max_radius:  # 超过最大查询半径，返回默认值
            return {
                "mean": mean,
                "grad_mean": 0,
                "slope": 0,
                "gradient": 0,
                "curvature": 0,
                "uncertainty": 1.0,  # 设定高不确定性
                "traversability": 0
            }



        if (nearest_x, nearest_y) not in self.data_dict:  # 额外检查，避免 KeyError
            return {
                "mean": mean,
                "grad_mean": 0,
                "slope": 0,
                "gradient": 0,
                "curvature": 0,
                "uncertainty": 1.0,
                "traversability": 0
            }


        return {
            "mean": mean,
            "grad_mean": grad_mean,
            "slope": slope,
            "gradient": step_height,
            "curvature": flatness,
            "uncertainty": uncertainty,
            "traversability": traversability
        }



        

