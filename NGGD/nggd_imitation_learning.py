# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - 主程序
实现完整的NGGD模仿学习流程：数据预处理 -> GMM建模 -> GMR回归 -> DMP泛化
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# 添加模块路径
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)

from data_preprocessing import TrajectoryPreprocessor
from gmm_trajectory_modeling import GMMTrajectoryModeling
from gmr_trajectory_regression import GMRTrajectoryRegression
from improved_dmp import ImprovedDMP

class NGGDImitationLearning:
    """NGGD模仿学习主类"""
    
    def __init__(self, n_gmm_components=10, n_dmp_bfs=100, noise_std=0.01):
        """
        初始化NGGD模仿学习系统
        
        Args:
            n_gmm_components: GMM组件数量
            n_dmp_bfs: DMP基函数数量
            noise_std: 噪声标准差
        """
        self.n_gmm_components = n_gmm_components
        self.n_dmp_bfs = n_dmp_bfs
        self.noise_std = noise_std
        
        # 初始化各个模块
        self.preprocessor = TrajectoryPreprocessor(noise_std=noise_std)
        self.gmm_modeler = GMMTrajectoryModeling(n_components=n_gmm_components)
        self.gmr_regressor = None
        self.improved_dmp = None
        
        # 存储处理结果
        self.processed_trajectories = None
        self.timestamps = None
        self.normalization_params = None
        self.gmr_trajectory = None
        self.reproduced_trajectory = None
        
    def learn_from_demonstrations(self, demonstration_trajectories, 
                                 alignment_method='linear', add_noise=True, 
                                 normalize=True, plot_results=True):
        """
        从示教轨迹学习
        
        Args:
            demonstration_trajectories: 示教轨迹列表
            alignment_method: 对齐方法
            add_noise: 是否添加噪声
            normalize: 是否归一化
            plot_results: 是否绘制结果
            
        Returns:
            learning_results: 学习结果字典
        """
        print("=" * 60)
        print("开始NGGD模仿学习流程")
        print("=" * 60)
        
        # 步骤1: 数据预处理
        print("\n步骤1: 示教数据预处理")
        print("-" * 30)
        self.processed_trajectories, self.timestamps, self.normalization_params = \
            self.preprocessor.preprocess_pipeline(
                demonstration_trajectories,
                alignment_method=alignment_method,
                add_noise=add_noise,
                normalize=normalize
            )
        
        # 步骤2: GMM轨迹建模
        print("\n步骤2: GMM运动轨迹建模")
        print("-" * 30)
        self.gmm_modeler.fit(self.processed_trajectories, self.timestamps)
        
        # 步骤3: GMR轨迹回归
        print("\n步骤3: GMR运动轨迹回归")
        print("-" * 30)
        self.gmr_regressor = GMRTrajectoryRegression(self.gmm_modeler)
        self.gmr_trajectory = self.gmr_regressor.regress(self.timestamps)
        
        # 步骤4: 改进DMP轨迹泛化
        print("\n步骤4: 基于DMP的轨迹泛化")
        print("-" * 30)
        n_dims = self.processed_trajectories.shape[2]
        self.improved_dmp = ImprovedDMP(n_dmps=n_dims, n_bfs=self.n_dmp_bfs)
        self.improved_dmp.learning_from_gmr(self.gmr_trajectory, self.timestamps)
        
        # 复现轨迹
        self.reproduced_trajectory, _, _ = self.improved_dmp.reproduce()
        
        print("\nNGGD模仿学习完成!")
        print(f"处理轨迹数量: {self.processed_trajectories.shape[0]}")
        print(f"轨迹维度: {self.processed_trajectories.shape[2]}")
        print(f"时间步数: {self.processed_trajectories.shape[1]}")
        
        # 准备学习结果
        learning_results = {
            'processed_trajectories': self.processed_trajectories,
            'timestamps': self.timestamps,
            'normalization_params': self.normalization_params,
            'gmm_model': self.gmm_modeler,
            'gmr_regressor': self.gmr_regressor,
            'gmr_trajectory': self.gmr_trajectory,
            'improved_dmp': self.improved_dmp,
            'reproduced_trajectory': self.reproduced_trajectory
        }
        
        # 可视化结果
        if plot_results:
            self.visualize_learning_results()
        
        return learning_results
    
    def generalize_trajectory(self, new_initial=None, new_goal=None, 
                            tau=1.0, plot_comparison=True):
        """
        轨迹泛化
        
        Args:
            new_initial: 新的初始位置
            new_goal: 新的目标位置
            tau: 时间缩放因子
            plot_comparison: 是否绘制比较图
            
        Returns:
            generalized_trajectory: 泛化后的轨迹
        """
        if self.improved_dmp is None:
            raise ValueError("请先执行learn_from_demonstrations方法")
        
        print(f"\n轨迹泛化 - 初始位置: {new_initial}, 目标位置: {new_goal}")
        print("-" * 50)
        
        # 执行泛化
        generalized_trajectory = self.improved_dmp.generalize(
            new_initial=new_initial, 
            new_goal=new_goal, 
            tau=tau
        )
        
        print(f"泛化轨迹形状: {generalized_trajectory.shape}")
        
        # 可视化比较
        if plot_comparison:
            self.visualize_generalization_comparison(generalized_trajectory)
        
        return generalized_trajectory
    
    def visualize_learning_results(self, save_dir=None):
        """可视化学习结果"""
        if save_dir is None:
            save_dir = os.path.join(current_dir, 'results')
        os.makedirs(save_dir, exist_ok=True)
        
        print("\n生成学习结果可视化...")
        
        # 1. 预处理结果可视化
        self.preprocessor.visualize_preprocessing(
            [traj for traj in self.processed_trajectories],
            self.processed_trajectories,
            save_path=os.path.join(save_dir, 'preprocessing_comparison.png')
        )
        
        # 2. GMM组件可视化
        self.gmm_modeler.visualize_gmm_components(
            save_path=os.path.join(save_dir, 'gmm_components.png')
        )
        
        # 3. GMR回归可视化
        self.gmr_regressor.visualize_regression(
            save_path=os.path.join(save_dir, 'gmr_regression.png')
        )
        
        # 4. 完整流程比较
        self.visualize_complete_pipeline(save_dir)
        
        print(f"可视化结果保存到: {save_dir}")
    
    def visualize_complete_pipeline(self, save_dir):
        """可视化完整流程"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # 原始示教轨迹
        ax1 = axes[0, 0]
        for i, traj in enumerate(self.processed_trajectories):
            ax1.plot(traj[:, 0], traj[:, 1], alpha=0.5, linewidth=1, 
                    label=f'Demo {i+1}' if i < 3 else "")
        ax1.set_title('Demonstration Trajectories')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.legend()
        ax1.grid(True)
        
        # GMR回归轨迹
        ax2 = axes[0, 1]
        ax2.plot(self.gmr_trajectory[:, 0], self.gmr_trajectory[:, 1], 
                'r-', linewidth=2, label='GMR Regression')
        ax2.set_title('GMR Regression')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.legend()
        ax2.grid(True)
        
        # DMP复现轨迹
        ax3 = axes[1, 0]
        ax3.plot(self.gmr_trajectory[:, 0], self.gmr_trajectory[:, 1], 
                'b-', linewidth=2, label='GMR Reference')
        ax3.plot(self.reproduced_trajectory[:, 0], self.reproduced_trajectory[:, 1], 
                'r--', linewidth=2, label='DMP Reproduction')
        ax3.set_title('DMP Reproduction')
        ax3.set_xlabel('X')
        ax3.set_ylabel('Y')
        ax3.legend()
        ax3.grid(True)
        
        # 轨迹比较
        ax4 = axes[1, 1]
        for i, traj in enumerate(self.processed_trajectories):
            ax4.plot(traj[:, 0], traj[:, 1], alpha=0.3, linewidth=1, color='blue')
        ax4.plot(self.gmr_trajectory[:, 0], self.gmr_trajectory[:, 1], 
                'g-', linewidth=2, label='GMR')
        ax4.plot(self.reproduced_trajectory[:, 0], self.reproduced_trajectory[:, 1], 
                'r--', linewidth=2, label='DMP')
        ax4.set_title('Complete Pipeline')
        ax4.set_xlabel('X')
        ax4.set_ylabel('Y')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, 'complete_pipeline.png'), 
                   dpi=300, bbox_inches='tight')
        plt.show()
    
    def visualize_generalization_comparison(self, generalized_trajectory, save_path=None):
        """可视化泛化比较"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 2D比较
        for i, traj in enumerate(self.processed_trajectories):
            ax1.plot(traj[:, 0], traj[:, 1], alpha=0.3, linewidth=1, color='blue')
        ax1.plot(self.gmr_trajectory[:, 0], self.gmr_trajectory[:, 1], 
                'g-', linewidth=2, label='GMR Reference')
        ax1.plot(self.reproduced_trajectory[:, 0], self.reproduced_trajectory[:, 1], 
                'r--', linewidth=2, label='DMP Reproduction')
        ax1.plot(generalized_trajectory[:, 0], generalized_trajectory[:, 1], 
                'm:', linewidth=3, label='DMP Generalization')
        ax1.set_title('2D Trajectory Comparison')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.legend()
        ax1.grid(True)
        
        # 3D比较（如果维度足够）
        if self.processed_trajectories.shape[2] >= 3:
            ax2 = fig.add_subplot(122, projection='3d')
            for i, traj in enumerate(self.processed_trajectories):
                ax2.plot(traj[:, 0], traj[:, 1], traj[:, 2], alpha=0.3, linewidth=1, color='blue')
            ax2.plot(self.gmr_trajectory[:, 0], self.gmr_trajectory[:, 1], self.gmr_trajectory[:, 2], 
                    'g-', linewidth=2, label='GMR Reference')
            ax2.plot(self.reproduced_trajectory[:, 0], self.reproduced_trajectory[:, 1], self.reproduced_trajectory[:, 2], 
                    'r--', linewidth=2, label='DMP Reproduction')
            ax2.plot(generalized_trajectory[:, 0], generalized_trajectory[:, 1], generalized_trajectory[:, 2], 
                    'm:', linewidth=3, label='DMP Generalization')
            ax2.set_title('3D Trajectory Comparison')
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            ax2.set_zlabel('Z')
            ax2.legend()
        else:
            ax2.axis('off')
            ax2.text(0.5, 0.5, '3D visualization\nnot available\n(dimension < 3)', 
                    ha='center', va='center', transform=ax2.transAxes)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"泛化比较图保存到: {save_path}")
        plt.show()
    
    def save_model(self, save_path):
        """保存模型"""
        import pickle
        
        model_data = {
            'n_gmm_components': self.n_gmm_components,
            'n_dmp_bfs': self.n_dmp_bfs,
            'noise_std': self.noise_std,
            'gmm_model': self.gmm_modeler,
            'gmr_regressor': self.gmr_regressor,
            'improved_dmp': self.improved_dmp,
            'normalization_params': self.normalization_params
        }
        
        with open(save_path, 'wb') as f:
            pickle.dump(model_data, f)
        
        print(f"模型保存到: {save_path}")
    
    def load_model(self, load_path):
        """加载模型"""
        import pickle
        
        with open(load_path, 'rb') as f:
            model_data = pickle.load(f)
        
        self.n_gmm_components = model_data['n_gmm_components']
        self.n_dmp_bfs = model_data['n_dmp_bfs']
        self.noise_std = model_data['noise_std']
        self.gmm_modeler = model_data['gmm_model']
        self.gmr_regressor = model_data['gmr_regressor']
        self.improved_dmp = model_data['improved_dmp']
        self.normalization_params = model_data['normalization_params']
        
        print(f"模型从 {load_path} 加载完成")

# 测试代码
if __name__ == "__main__":
    # 生成测试示教数据
    np.random.seed(42)
    n_demonstrations = 8
    n_timesteps = 100
    n_dims = 3
    
    print("生成测试示教数据...")
    demonstration_trajectories = []
    
    for i in range(n_demonstrations):
        # 生成不同的轨迹形状
        t = np.linspace(0, 2*np.pi, n_timesteps)
        
        if i % 3 == 0:  # 圆形轨迹
            trajectory = np.column_stack([
                np.sin(t) + np.random.normal(0, 0.05, n_timesteps),
                np.cos(t) + np.random.normal(0, 0.05, n_timesteps),
                t/(2*np.pi) + np.random.normal(0, 0.02, n_timesteps)
            ])
        elif i % 3 == 1:  # 8字形轨迹
            trajectory = np.column_stack([
                np.sin(t) + np.random.normal(0, 0.05, n_timesteps),
                np.sin(2*t) + np.random.normal(0, 0.05, n_timesteps),
                t/(2*np.pi) + np.random.normal(0, 0.02, n_timesteps)
            ])
        else:  # 螺旋轨迹
            trajectory = np.column_stack([
                t/(2*np.pi) * np.sin(t) + np.random.normal(0, 0.05, n_timesteps),
                t/(2*np.pi) * np.cos(t) + np.random.normal(0, 0.05, n_timesteps),
                t/(2*np.pi) + np.random.normal(0, 0.02, n_timesteps)
            ])
        
        demonstration_trajectories.append(trajectory)
    
    # 创建NGGD模仿学习系统
    nggd_system = NGGDImitationLearning(
        n_gmm_components=12,
        n_dmp_bfs=80,
        noise_std=0.02
    )
    
    # 执行学习
    learning_results = nggd_system.learn_from_demonstrations(
        demonstration_trajectories,
        alignment_method='linear',
        add_noise=True,
        normalize=True,
        plot_results=True
    )
    
    # 测试泛化能力
    print("\n测试轨迹泛化能力...")
    
    # 泛化1: 不同的初始和目标位置
    generalized_traj1 = nggd_system.generalize_trajectory(
        new_initial=[0.5, 0.5, 0.2],
        new_goal=[-0.5, -0.5, 0.8],
        tau=1.0
    )
    
    # 泛化2: 时间缩放
    generalized_traj2 = nggd_system.generalize_trajectory(
        new_initial=[0.2, 0.8, 0.1],
        new_goal=[0.8, 0.2, 0.9],
        tau=0.5
    )
    
    # 泛化3: 另一个不同的轨迹
    generalized_traj3 = nggd_system.generalize_trajectory(
        new_initial=[-0.3, 0.7, 0.3],
        new_goal=[0.7, -0.3, 0.7],
        tau=1.2
    )
    
    print("\nNGGD模仿学习测试完成!")
    print(f"示教轨迹数量: {len(demonstration_trajectories)}")
    print(f"GMM组件数量: {nggd_system.n_gmm_components}")
    print(f"DMP基函数数量: {nggd_system.n_dmp_bfs}")
    print(f"轨迹维度: {n_dims}")
