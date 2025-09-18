# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - GMM轨迹建模模块
实现高斯混合模型对示教轨迹的建模
"""

import numpy as np
try:
    from sklearn.mixture import GaussianMixture
except ImportError:
    print("警告: 未安装scikit-learn，请运行: pip install scikit-learn")
    GaussianMixture = None
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class GMMTrajectoryModeling:
    """GMM轨迹建模类"""
    
    def __init__(self, n_components=10, random_state=42):
        """
        初始化GMM轨迹建模器
        
        Args:
            n_components: GMM组件数量
            random_state: 随机种子
        """
        self.n_components = n_components
        self.random_state = random_state
        self.gmm = None
        self.trajectories = None
        self.timestamps = None
        self.n_dims = None
        self.n_trajectories = None
        
    def fit(self, trajectories, timestamps=None):
        """
        使用GMM学习示教轨迹中的特征信息
        
        Args:
            trajectories: 示教轨迹数据，形状为 (n_trajectories, n_timesteps, n_dims)
            timestamps: 时间戳，形状为 (n_timesteps,)
        """
        self.trajectories = np.array(trajectories)
        self.n_trajectories, self.n_timesteps, self.n_dims = self.trajectories.shape
        
        if timestamps is None:
            self.timestamps = np.linspace(0, 1, self.n_timesteps)
        else:
            self.timestamps = np.array(timestamps)
            
        # 将轨迹数据重塑为 (n_samples, n_features)
        # 每个样本包含位置和时间信息
        n_samples = self.n_trajectories * self.n_timesteps
        n_features = self.n_dims + 1  # 添加时间维度
        
        data = np.zeros((n_samples, n_features))
        
        for i in range(self.n_trajectories):
            for t in range(self.n_timesteps):
                idx = i * self.n_timesteps + t
                data[idx, :self.n_dims] = self.trajectories[i, t, :]
                data[idx, self.n_dims] = self.timestamps[t]
        
        # 训练GMM模型
        self.gmm = GaussianMixture(
            n_components=self.n_components,
            random_state=self.random_state,
            covariance_type='full'
        )
        self.gmm.fit(data)
        
        print(f"GMM训练完成，组件数量: {self.n_components}")
        print(f"轨迹数量: {self.n_trajectories}, 时间步数: {self.n_timesteps}, 维度: {self.n_dims}")
        
    def get_joint_probability_density(self, positions, times):
        """
        计算联合概率密度
        
        Args:
            positions: 位置数据，形状为 (n_samples, n_dims)
            times: 时间数据，形状为 (n_samples,)
            
        Returns:
            probability_density: 联合概率密度
        """
        if self.gmm is None:
            raise ValueError("GMM模型尚未训练，请先调用fit方法")
            
        # 准备数据
        n_samples = positions.shape[0]
        data = np.zeros((n_samples, self.n_dims + 1))
        data[:, :self.n_dims] = positions
        data[:, self.n_dims] = times
        
        # 计算概率密度
        probability_density = np.exp(self.gmm.score_samples(data))
        
        return probability_density
    
    def sample_trajectory(self, n_samples=None):
        """
        从GMM中采样轨迹
        
        Args:
            n_samples: 采样数量，默认为训练时的时间步数
            
        Returns:
            sampled_trajectory: 采样的轨迹
        """
        if self.gmm is None:
            raise ValueError("GMM模型尚未训练，请先调用fit方法")
            
        if n_samples is None:
            n_samples = self.n_timesteps
            
        # 从GMM中采样
        sampled_data, _ = self.gmm.sample(n_samples)
        
        # 分离位置和时间信息
        sampled_trajectory = sampled_data[:, :self.n_dims]
        sampled_times = sampled_data[:, self.n_dims]
        
        # 按时间排序
        sort_indices = np.argsort(sampled_times)
        sampled_trajectory = sampled_trajectory[sort_indices]
        sampled_times = sampled_times[sort_indices]
        
        return sampled_trajectory, sampled_times
    
    def get_means_and_covariances(self):
        """
        获取GMM的均值和协方差矩阵
        
        Returns:
            means: 均值矩阵
            covariances: 协方差矩阵
        """
        if self.gmm is None:
            raise ValueError("GMM模型尚未训练，请先调用fit方法")
            
        return self.gmm.means_, self.gmm.covariances_
    
    def visualize_gmm_components(self, dim1=0, dim2=1, save_path=None):
        """
        可视化GMM组件
        
        Args:
            dim1: 第一个维度索引
            dim2: 第二个维度索引
            save_path: 保存路径
        """
        if self.gmm is None:
            raise ValueError("GMM模型尚未训练，请先调用fit方法")
            
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # 绘制原始轨迹
        for i in range(self.n_trajectories):
            ax.plot(self.trajectories[i, :, dim1], 
                   self.trajectories[i, :, dim2], 
                   'b-', alpha=0.3, linewidth=1)
        
        # 绘制GMM组件
        means = self.gmm.means_
        covariances = self.gmm.covariances_
        
        for i in range(self.n_components):
            mean = means[i, [dim1, dim2]]
            cov = covariances[i, [dim1, dim2], :][:, [dim1, dim2]]
            
            # 绘制椭圆
            from matplotlib.patches import Ellipse
            eigenvals, eigenvecs = np.linalg.eigh(cov)
            angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
            width, height = 2 * np.sqrt(eigenvals)
            
            ellipse = Ellipse(mean, width, height, angle=angle, 
                            alpha=0.3, color=f'C{i}')
            ax.add_patch(ellipse)
            
            # 绘制中心点
            ax.scatter(mean[0], mean[1], c=f'C{i}', s=100, marker='x')
        
        ax.set_xlabel(f'Dimension {dim1}')
        ax.set_ylabel(f'Dimension {dim2}')
        ax.set_title('GMM Components Visualization')
        ax.grid(True)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"GMM组件可视化图保存到: {save_path}")
        
        plt.show()
    
    def visualize_3d_trajectories(self, save_path=None):
        """
        3D轨迹可视化
        
        Args:
            save_path: 保存路径
        """
        if self.n_dims < 3:
            print("轨迹维度小于3，无法进行3D可视化")
            return
            
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制原始轨迹
        for i in range(self.n_trajectories):
            ax.plot(self.trajectories[i, :, 0], 
                   self.trajectories[i, :, 1], 
                   self.trajectories[i, :, 2], 
                   'b-', alpha=0.3, linewidth=1)
        
        # 绘制GMM中心点
        means = self.gmm.means_
        ax.scatter(means[:, 0], means[:, 1], means[:, 2], 
                  c='red', s=100, marker='x', label='GMM Centers')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Trajectory Visualization with GMM Centers')
        ax.legend()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"3D轨迹可视化图保存到: {save_path}")
        
        plt.show()

# 测试代码
if __name__ == "__main__":
    # 生成测试数据
    np.random.seed(42)
    n_trajectories = 5
    n_timesteps = 100
    n_dims = 3
    
    # 生成多条示教轨迹
    trajectories = []
    for i in range(n_trajectories):
        t = np.linspace(0, 2*np.pi, n_timesteps)
        # 添加一些变化
        noise = np.random.normal(0, 0.1, (n_timesteps, n_dims))
        trajectory = np.column_stack([
            np.sin(t) + noise[:, 0],
            np.cos(t) + noise[:, 1],
            t/(2*np.pi) + noise[:, 2]
        ])
        trajectories.append(trajectory)
    
    trajectories = np.array(trajectories)
    
    # 创建GMM建模器
    gmm_modeler = GMMTrajectoryModeling(n_components=8)
    
    # 训练GMM
    gmm_modeler.fit(trajectories)
    
    # 可视化
    gmm_modeler.visualize_gmm_components(dim1=0, dim2=1)
    gmm_modeler.visualize_3d_trajectories()
    
    # 采样新轨迹
    sampled_traj, sampled_times = gmm_modeler.sample_trajectory()
    print(f"采样轨迹形状: {sampled_traj.shape}")
    print(f"采样时间形状: {sampled_times.shape}")
