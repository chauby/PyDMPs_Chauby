# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - 数据预处理模块
实现轨迹时间对齐和添加高斯噪声等预处理操作
"""

import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryPreprocessor:
    """轨迹数据预处理类"""
    
    def __init__(self, target_length=None, noise_std=0.01):
        """
        初始化轨迹预处理器
        
        Args:
            target_length: 目标轨迹长度，如果为None则使用最长轨迹的长度
            noise_std: 高斯噪声标准差
        """
        self.target_length = target_length
        self.noise_std = noise_std
        
    def time_alignment(self, trajectories, method='dtw'):
        """
        轨迹时间对齐
        
        Args:
            trajectories: 轨迹列表，每个轨迹形状为 (n_timesteps, n_dims)
            method: 对齐方法 ('dtw', 'linear', 'uniform')
            
        Returns:
            aligned_trajectories: 对齐后的轨迹
            aligned_timestamps: 对齐后的时间戳
        """
        trajectories = np.array(trajectories)
        n_trajectories, n_timesteps, n_dims = trajectories.shape
        
        if method == 'dtw':
            return self._dtw_alignment(trajectories)
        elif method == 'linear':
            return self._linear_alignment(trajectories)
        elif method == 'uniform':
            return self._uniform_alignment(trajectories)
        else:
            raise ValueError(f"不支持的对齐方法: {method}")
    
    def _dtw_alignment(self, trajectories):
        """
        使用动态时间规整(DTW)进行轨迹对齐
        
        Args:
            trajectories: 轨迹数据
            
        Returns:
            aligned_trajectories: 对齐后的轨迹
            aligned_timestamps: 对齐后的时间戳
        """
        n_trajectories, n_timesteps, n_dims = trajectories.shape
        
        # 选择参考轨迹（最长的轨迹）
        ref_idx = np.argmax([len(traj) for traj in trajectories])
        reference_trajectory = trajectories[ref_idx]
        
        # 计算DTW路径
        aligned_trajectories = []
        for i, trajectory in enumerate(trajectories):
            if i == ref_idx:
                aligned_trajectories.append(reference_trajectory)
                continue
                
            # 简化的DTW对齐
            aligned_traj = self._simple_dtw_align(trajectory, reference_trajectory)
            aligned_trajectories.append(aligned_traj)
        
        # 生成统一的时间戳
        aligned_timestamps = np.linspace(0, 1, len(reference_trajectory))
        
        return np.array(aligned_trajectories), aligned_timestamps
    
    def _simple_dtw_align(self, trajectory, reference):
        """
        简化的DTW对齐算法
        
        Args:
            trajectory: 待对齐轨迹
            reference: 参考轨迹
            
        Returns:
            aligned_trajectory: 对齐后的轨迹
        """
        n_ref = len(reference)
        n_traj = len(trajectory)
        
        # 计算距离矩阵
        distances = cdist(trajectory, reference, metric='euclidean')
        
        # 动态规划找最优路径
        dp = np.full((n_traj + 1, n_ref + 1), np.inf)
        dp[0, 0] = 0
        
        for i in range(1, n_traj + 1):
            for j in range(1, n_ref + 1):
                dp[i, j] = distances[i-1, j-1] + min(dp[i-1, j], dp[i, j-1], dp[i-1, j-1])
        
        # 回溯找路径
        path = []
        i, j = n_traj, n_ref
        while i > 0 and j > 0:
            path.append((i-1, j-1))
            if dp[i-1, j] <= dp[i, j-1] and dp[i-1, j] <= dp[i-1, j-1]:
                i -= 1
            elif dp[i, j-1] <= dp[i-1, j-1]:
                j -= 1
            else:
                i -= 1
                j -= 1
        
        path.reverse()
        
        # 根据路径生成对齐轨迹
        aligned_trajectory = np.zeros((n_ref, trajectory.shape[1]))
        for ref_idx in range(n_ref):
            # 找到所有映射到当前参考点的轨迹点
            traj_indices = [traj_idx for traj_idx, ref_idx_path in path if ref_idx_path == ref_idx]
            
            if traj_indices:
                # 使用平均位置
                aligned_trajectory[ref_idx] = np.mean(trajectory[traj_indices], axis=0)
            else:
                # 如果没有映射，使用最近的点
                if ref_idx > 0:
                    aligned_trajectory[ref_idx] = aligned_trajectory[ref_idx-1]
                else:
                    aligned_trajectory[ref_idx] = trajectory[0]
        
        return aligned_trajectory
    
    def _linear_alignment(self, trajectories):
        """
        线性插值对齐
        
        Args:
            trajectories: 轨迹数据
            
        Returns:
            aligned_trajectories: 对齐后的轨迹
            aligned_timestamps: 对齐后的时间戳
        """
        n_trajectories, n_timesteps, n_dims = trajectories.shape
        
        if self.target_length is None:
            target_length = n_timesteps
        else:
            target_length = self.target_length
        
        aligned_trajectories = []
        for trajectory in trajectories:
            # 创建原始时间轴
            original_times = np.linspace(0, 1, len(trajectory))
            target_times = np.linspace(0, 1, target_length)
            
            # 对每个维度进行插值
            aligned_traj = np.zeros((target_length, n_dims))
            for dim in range(n_dims):
                interp_func = interp1d(original_times, trajectory[:, dim], 
                                     kind='linear', fill_value='extrapolate')
                aligned_traj[:, dim] = interp_func(target_times)
            
            aligned_trajectories.append(aligned_traj)
        
        aligned_timestamps = np.linspace(0, 1, target_length)
        return np.array(aligned_trajectories), aligned_timestamps
    
    def _uniform_alignment(self, trajectories):
        """
        均匀时间对齐
        
        Args:
            trajectories: 轨迹数据
            
        Returns:
            aligned_trajectories: 对齐后的轨迹
            aligned_timestamps: 对齐后的时间戳
        """
        n_trajectories, n_timesteps, n_dims = trajectories.shape
        
        if self.target_length is None:
            target_length = n_timesteps
        else:
            target_length = self.target_length
        
        aligned_trajectories = []
        for trajectory in trajectories:
            # 均匀采样
            indices = np.linspace(0, len(trajectory)-1, target_length, dtype=int)
            aligned_traj = trajectory[indices]
            aligned_trajectories.append(aligned_traj)
        
        aligned_timestamps = np.linspace(0, 1, target_length)
        return np.array(aligned_trajectories), aligned_timestamps
    
    def add_gaussian_noise(self, trajectories, noise_std=None):
        """
        添加高斯噪声
        
        Args:
            trajectories: 轨迹数据
            noise_std: 噪声标准差，如果为None则使用初始化时的值
            
        Returns:
            noisy_trajectories: 添加噪声后的轨迹
        """
        if noise_std is None:
            noise_std = self.noise_std
        
        trajectories = np.array(trajectories)
        noisy_trajectories = trajectories.copy()
        
        # 添加高斯噪声
        noise = np.random.normal(0, noise_std, trajectories.shape)
        noisy_trajectories += noise
        
        return noisy_trajectories
    
    def normalize_trajectories(self, trajectories, method='minmax'):
        """
        轨迹归一化
        
        Args:
            trajectories: 轨迹数据
            method: 归一化方法 ('minmax', 'zscore')
            
        Returns:
            normalized_trajectories: 归一化后的轨迹
            normalization_params: 归一化参数
        """
        trajectories = np.array(trajectories)
        n_trajectories, n_timesteps, n_dims = trajectories.shape
        
        if method == 'minmax':
            # Min-Max归一化
            min_vals = np.min(trajectories, axis=(0, 1))
            max_vals = np.max(trajectories, axis=(0, 1))
            
            normalized_trajectories = (trajectories - min_vals) / (max_vals - min_vals)
            
            normalization_params = {
                'method': 'minmax',
                'min_vals': min_vals,
                'max_vals': max_vals
            }
            
        elif method == 'zscore':
            # Z-score归一化
            mean_vals = np.mean(trajectories, axis=(0, 1))
            std_vals = np.std(trajectories, axis=(0, 1))
            
            normalized_trajectories = (trajectories - mean_vals) / std_vals
            
            normalization_params = {
                'method': 'zscore',
                'mean_vals': mean_vals,
                'std_vals': std_vals
            }
        
        else:
            raise ValueError(f"不支持的归一化方法: {method}")
        
        return normalized_trajectories, normalization_params
    
    def denormalize_trajectories(self, normalized_trajectories, normalization_params):
        """
        反归一化轨迹
        
        Args:
            normalized_trajectories: 归一化后的轨迹
            normalization_params: 归一化参数
            
        Returns:
            denormalized_trajectories: 反归一化后的轨迹
        """
        if normalization_params['method'] == 'minmax':
            denormalized_trajectories = (normalized_trajectories * 
                                       (normalization_params['max_vals'] - normalization_params['min_vals']) + 
                                       normalization_params['min_vals'])
        elif normalization_params['method'] == 'zscore':
            denormalized_trajectories = (normalized_trajectories * 
                                       normalization_params['std_vals'] + 
                                       normalization_params['mean_vals'])
        else:
            raise ValueError(f"不支持的归一化方法: {normalization_params['method']}")
        
        return denormalized_trajectories
    
    def preprocess_pipeline(self, trajectories, alignment_method='linear', 
                          add_noise=True, normalize=True, noise_std=None):
        """
        完整的预处理流水线
        
        Args:
            trajectories: 原始轨迹数据
            alignment_method: 对齐方法
            add_noise: 是否添加噪声
            normalize: 是否归一化
            noise_std: 噪声标准差
            
        Returns:
            processed_trajectories: 处理后的轨迹
            timestamps: 时间戳
            normalization_params: 归一化参数
        """
        print("开始轨迹预处理...")
        
        # 1. 时间对齐
        print("1. 进行时间对齐...")
        aligned_trajectories, timestamps = self.time_alignment(trajectories, method=alignment_method)
        
        # 2. 添加噪声
        if add_noise:
            print("2. 添加高斯噪声...")
            if noise_std is not None:
                self.noise_std = noise_std
            aligned_trajectories = self.add_gaussian_noise(aligned_trajectories)
        
        # 3. 归一化
        normalization_params = None
        if normalize:
            print("3. 进行归一化...")
            aligned_trajectories, normalization_params = self.normalize_trajectories(aligned_trajectories)
        
        print("轨迹预处理完成!")
        print(f"处理后轨迹形状: {aligned_trajectories.shape}")
        
        return aligned_trajectories, timestamps, normalization_params
    
    def visualize_preprocessing(self, original_trajectories, processed_trajectories, 
                              timestamps=None, dim1=0, dim2=1, save_path=None):
        """
        可视化预处理结果
        
        Args:
            original_trajectories: 原始轨迹
            processed_trajectories: 处理后的轨迹
            timestamps: 时间戳
            dim1: 第一个维度索引
            dim2: 第二个维度索引
            save_path: 保存路径
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 原始轨迹
        for i, trajectory in enumerate(original_trajectories):
            ax1.plot(trajectory[:, dim1], trajectory[:, dim2], 
                    alpha=0.7, linewidth=1, label=f'Traj {i+1}' if i < 5 else "")
        
        ax1.set_xlabel(f'Dimension {dim1}')
        ax1.set_ylabel(f'Dimension {dim2}')
        ax1.set_title('Original Trajectories')
        ax1.legend()
        ax1.grid(True)
        
        # 处理后的轨迹
        for i, trajectory in enumerate(processed_trajectories):
            ax2.plot(trajectory[:, dim1], trajectory[:, dim2], 
                    alpha=0.7, linewidth=1, label=f'Traj {i+1}' if i < 5 else "")
        
        ax2.set_xlabel(f'Dimension {dim1}')
        ax2.set_ylabel(f'Dimension {dim2}')
        ax2.set_title('Preprocessed Trajectories')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"预处理可视化图保存到: {save_path}")
        
        plt.show()

# 测试代码
if __name__ == "__main__":
    # 生成测试数据
    np.random.seed(42)
    n_trajectories = 5
    n_dims = 3
    
    # 生成不同长度的轨迹
    trajectories = []
    for i in range(n_trajectories):
        n_timesteps = np.random.randint(80, 120)
        t = np.linspace(0, 2*np.pi, n_timesteps)
        noise = np.random.normal(0, 0.05, (n_timesteps, n_dims))
        trajectory = np.column_stack([
            np.sin(t) + noise[:, 0],
            np.cos(t) + noise[:, 1],
            t/(2*np.pi) + noise[:, 2]
        ])
        trajectories.append(trajectory)
    
    # 创建预处理器
    preprocessor = TrajectoryPreprocessor(target_length=100, noise_std=0.02)
    
    # 执行预处理
    processed_trajectories, timestamps, norm_params = preprocessor.preprocess_pipeline(
        trajectories, alignment_method='linear', add_noise=True, normalize=True
    )
    
    # 可视化
    preprocessor.visualize_preprocessing(trajectories, processed_trajectories)
    
    print(f"原始轨迹数量: {len(trajectories)}")
    print(f"原始轨迹长度: {[len(traj) for traj in trajectories]}")
    print(f"处理后轨迹形状: {processed_trajectories.shape}")
    print(f"时间戳长度: {len(timestamps)}")
