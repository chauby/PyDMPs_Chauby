# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - GMR轨迹回归模块
实现高斯混合回归，得到概率最优的回归轨迹
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class GMRTrajectoryRegression:
    """GMR轨迹回归类"""
    
    def __init__(self, gmm_model):
        """
        初始化GMR回归器
        
        Args:
            gmm_model: 训练好的GMM模型
        """
        self.gmm = gmm_model
        self.n_components = gmm_model.n_components
        self.n_dims = gmm_model.n_dims
        
    def regress(self, query_times, return_covariance=False):
        """
        通过GMR进行轨迹回归
        
        Args:
            query_times: 查询时间点，形状为 (n_query,)
            return_covariance: 是否返回协方差
            
        Returns:
            regressed_trajectory: 回归轨迹，形状为 (n_query, n_dims)
            covariance: 协方差矩阵（如果return_covariance=True）
        """
        query_times = np.array(query_times).reshape(-1, 1)
        n_query = len(query_times)
        
        # 获取GMM参数
        means = self.gmm.means_
        covariances = self.gmm.covariances_
        weights = self.gmm.weights_
        
        # 分离位置和时间相关的均值和协方差
        means_pos = means[:, :self.n_dims]  # 位置均值
        means_time = means[:, self.n_dims:]  # 时间均值
        
        cov_pos_pos = covariances[:, :self.n_dims, :self.n_dims]  # 位置-位置协方差
        cov_pos_time = covariances[:, :self.n_dims, self.n_dims:]  # 位置-时间协方差
        cov_time_pos = covariances[:, self.n_dims:, :self.n_dims]  # 时间-位置协方差
        cov_time_time = covariances[:, self.n_dims:, self.n_dims:]  # 时间-时间协方差
        
        # 初始化输出
        regressed_trajectory = np.zeros((n_query, self.n_dims))
        if return_covariance:
            trajectory_covariance = np.zeros((n_query, self.n_dims, self.n_dims))
        
        for t_idx, t in enumerate(query_times):
            # 计算每个组件的后验概率
            posterior_probs = np.zeros(self.n_components)
            
            for k in range(self.n_components):
                # 计算时间概率密度
                time_diff = t - means_time[k]
                time_var = cov_time_time[k, 0, 0]
                time_prob = np.exp(-0.5 * (time_diff**2) / time_var) / np.sqrt(2 * np.pi * time_var)
                
                # 计算权重
                posterior_probs[k] = weights[k] * time_prob
            
            # 归一化
            posterior_probs = posterior_probs / np.sum(posterior_probs)
            
            # 计算回归结果
            regressed_pos = np.zeros(self.n_dims)
            
            for k in range(self.n_components):
                if posterior_probs[k] > 1e-10:  # 避免数值问题
                    # 条件均值
                    time_diff = t - means_time[k]
                    cov_pos_time_k = cov_pos_time[k, :, 0]
                    cov_time_time_k = cov_time_time[k, 0, 0]
                    
                    conditional_mean = means_pos[k] + cov_pos_time_k * time_diff / cov_time_time_k
                    regressed_pos += posterior_probs[k] * conditional_mean
            
            regressed_trajectory[t_idx] = regressed_pos
            
            # 计算协方差（如果需要）
            if return_covariance:
                cov_pos = np.zeros((self.n_dims, self.n_dims))
                
                for k in range(self.n_components):
                    if posterior_probs[k] > 1e-10:
                        # 条件协方差
                        cov_pos_pos_k = cov_pos_pos[k]
                        cov_pos_time_k = cov_pos_time[k, :, 0]
                        cov_time_pos_k = cov_time_pos[k, 0, :]
                        cov_time_time_k = cov_time_time[k, 0, 0]
                        
                        conditional_cov = cov_pos_pos_k - np.outer(cov_pos_time_k, cov_time_pos_k) / cov_time_time_k
                        
                        # 条件均值
                        time_diff = t - means_time[k]
                        conditional_mean = means_pos[k] + cov_pos_time_k * time_diff / cov_time_time_k
                        
                        # 加权协方差
                        mean_diff = regressed_pos - conditional_mean
                        cov_pos += posterior_probs[k] * (conditional_cov + np.outer(mean_diff, mean_diff))
                
                trajectory_covariance[t_idx] = cov_pos
        
        if return_covariance:
            return regressed_trajectory, trajectory_covariance
        else:
            return regressed_trajectory
    
    def regress_with_confidence(self, query_times, confidence_level=0.95):
        """
        带置信区间的轨迹回归
        
        Args:
            query_times: 查询时间点
            confidence_level: 置信水平
            
        Returns:
            regressed_trajectory: 回归轨迹
            lower_bound: 下界
            upper_bound: 上界
        """
        regressed_trajectory, covariance = self.regress(query_times, return_covariance=True)
        
        # 计算置信区间
        from scipy.stats import chi2
        chi2_val = chi2.ppf(confidence_level, self.n_dims)
        
        n_query = len(query_times)
        lower_bound = np.zeros_like(regressed_trajectory)
        upper_bound = np.zeros_like(regressed_trajectory)
        
        for t_idx in range(n_query):
            cov_t = covariance[t_idx]
            eigenvals, eigenvecs = np.linalg.eigh(cov_t)
            
            # 计算置信椭球的主轴长度
            radius = np.sqrt(chi2_val * eigenvals)
            
            # 计算每个维度的置信区间
            for dim in range(self.n_dims):
                std_dev = np.sqrt(np.sum((eigenvecs[:, dim] * radius)**2))
                lower_bound[t_idx, dim] = regressed_trajectory[t_idx, dim] - std_dev
                upper_bound[t_idx, dim] = regressed_trajectory[t_idx, dim] + std_dev
        
        return regressed_trajectory, lower_bound, upper_bound
    
    def visualize_regression(self, query_times=None, confidence_level=0.95, 
                           dim1=0, dim2=1, save_path=None):
        """
        可视化回归结果
        
        Args:
            query_times: 查询时间点
            confidence_level: 置信水平
            dim1: 第一个维度索引
            dim2: 第二个维度索引
            save_path: 保存路径
        """
        if query_times is None:
            query_times = np.linspace(0, 1, 100)
        
        # 进行回归
        regressed_traj, lower_bound, upper_bound = self.regress_with_confidence(
            query_times, confidence_level
        )
        
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # 绘制原始轨迹
        for i in range(self.gmm.n_trajectories):
            ax.plot(self.gmm.trajectories[i, :, dim1], 
                   self.gmm.trajectories[i, :, dim2], 
                   'b-', alpha=0.3, linewidth=1, label='Demonstration' if i == 0 else "")
        
        # 绘制回归轨迹
        ax.plot(regressed_traj[:, dim1], regressed_traj[:, dim2], 
               'r-', linewidth=2, label='GMR Regression')
        
        # 绘制置信区间
        ax.fill_between(regressed_traj[:, dim1], 
                       lower_bound[:, dim1], 
                       upper_bound[:, dim1], 
                       alpha=0.2, color='red', label=f'{confidence_level*100}% Confidence')
        
        ax.set_xlabel(f'Dimension {dim1}')
        ax.set_ylabel(f'Dimension {dim2}')
        ax.set_title('GMR Trajectory Regression')
        ax.legend()
        ax.grid(True)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"GMR回归可视化图保存到: {save_path}")
        
        plt.show()
    
    def visualize_3d_regression(self, query_times=None, save_path=None):
        """
        3D回归可视化
        
        Args:
            query_times: 查询时间点
            save_path: 保存路径
        """
        if self.n_dims < 3:
            print("轨迹维度小于3，无法进行3D可视化")
            return
            
        if query_times is None:
            query_times = np.linspace(0, 1, 100)
        
        # 进行回归
        regressed_traj = self.regress(query_times)
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制原始轨迹
        for i in range(self.gmm.n_trajectories):
            ax.plot(self.gmm.trajectories[i, :, 0], 
                   self.gmm.trajectories[i, :, 1], 
                   self.gmm.trajectories[i, :, 2], 
                   'b-', alpha=0.3, linewidth=1, label='Demonstration' if i == 0 else "")
        
        # 绘制回归轨迹
        ax.plot(regressed_traj[:, 0], regressed_traj[:, 1], regressed_traj[:, 2], 
               'r-', linewidth=2, label='GMR Regression')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D GMR Trajectory Regression')
        ax.legend()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"3D GMR回归可视化图保存到: {save_path}")
        
        plt.show()

# 测试代码
if __name__ == "__main__":
    from gmm_trajectory_modeling import GMMTrajectoryModeling
    
    # 生成测试数据
    np.random.seed(42)
    n_trajectories = 5
    n_timesteps = 100
    n_dims = 3
    
    trajectories = []
    for i in range(n_trajectories):
        t = np.linspace(0, 2*np.pi, n_timesteps)
        noise = np.random.normal(0, 0.1, (n_timesteps, n_dims))
        trajectory = np.column_stack([
            np.sin(t) + noise[:, 0],
            np.cos(t) + noise[:, 1],
            t/(2*np.pi) + noise[:, 2]
        ])
        trajectories.append(trajectory)
    
    trajectories = np.array(trajectories)
    
    # 训练GMM
    gmm_modeler = GMMTrajectoryModeling(n_components=8)
    gmm_modeler.fit(trajectories)
    
    # 创建GMR回归器
    gmr_regressor = GMRTrajectoryRegression(gmm_modeler)
    
    # 进行回归
    query_times = np.linspace(0, 1, 100)
    regressed_traj = gmr_regressor.regress(query_times)
    
    print(f"回归轨迹形状: {regressed_traj.shape}")
    
    # 可视化
    gmr_regressor.visualize_regression(query_times)
    gmr_regressor.visualize_3d_regression(query_times)
