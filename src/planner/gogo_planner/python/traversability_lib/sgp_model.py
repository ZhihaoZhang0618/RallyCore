#!/usr/bin/env python3
import torch
import gpytorch

class SGPModel(gpytorch.models.ExactGP):
    """
    高斯过程回归模型（SGPModel）
    使用高斯过程（Gaussian Process）对点云数据进行建模，用于预测每个点的高度值和不确定性。
    在论文中，作者使用了精确高斯过程（Exact GP），这个模型通过均值函数和协方差函数对数据进行建模。
    """
    def __init__(self, train_x, train_y, likelihood, inducing_points, lengthscale=0.7, alpha=10):
        """
        初始化高斯过程回归模型。
        参数:
        - train_x: 训练数据的输入（点的坐标）
        - train_y: 训练数据的输出（点的高度）
        - inducing_points: 诱导点的数量
        - lengthscale, alpha: RQKernel（理性二次核函数）的超参数
        """
        super(SGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()  # 常数均值函数
        inducing_variable=train_x

        # 使用 RQKernel（Rational Quadratic Kernel）
        self.base_covar_module = gpytorch.kernels.ScaleKernel(
            gpytorch.kernels.RQKernel(lengthscale=torch.tensor([lengthscale, lengthscale]), alpha=torch.tensor([alpha]))
        )
        self.covar_module = gpytorch.kernels.InducingPointKernel(self.base_covar_module, inducing_points=inducing_variable, likelihood=likelihood)

        # 判断是否使用GPU，如果有GPU，则将模型加载到GPU上
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    def forward(self, x):
        """
        前向传播，计算输入点的均值和协方差，返回多元正态分布。
        """
        mean_x = self.mean_module(x).squeeze()    # 均值函数
        covar_x = self.covar_module(x)  # 协方差函数
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)  # 返回多元正态分布