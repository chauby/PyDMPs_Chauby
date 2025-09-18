# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - 改进的DMP算法模块
基于GMR回归轨迹的改进动态运动基元算法
"""

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# 添加DMP模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'DMP'))
from cs import CanonicalSystem

class ImprovedDMP:
    """改进的DMP类，基于GMR回归轨迹"""
    
    def __init__(self, n_dmps=1, n_bfs=100, dt=0.01, alpha_y=None, beta_y=None, 
                 gmr_trajectory=None, **kwargs):
        """
        初始化改进的DMP
        
        Args:
            n_dmps: 数据维度数
            n_bfs: 基函数数量
            dt: 时间步长
            alpha_y: 阻尼参数
            beta_y: 刚度参数
            gmr_trajectory: GMR回归的轨迹
        """
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt
        self.gmr_trajectory = gmr_trajectory
        
        # DMP参数
        alpha_y_tmp = 60 if alpha_y is None else alpha_y
        beta_y_tmp = alpha_y_tmp / 4.0 if beta_y is None else beta_y
        self.alpha_y = np.ones(n_dmps) * alpha_y_tmp
        self.beta_y = np.ones(n_dmps) * beta_y_tmp
        self.tau = 1.0
        
        # 状态变量
        self.y0 = np.zeros(n_dmps)
        self.goal = np.ones(n_dmps)
        self.y = self.y0.copy()
        self.dy = np.zeros(n_dmps)
        self.ddy = np.zeros(n_dmps)
        
        # 强制项参数
        self.w = np.zeros((n_dmps, n_bfs))
        self.psi_centers = np.zeros(n_bfs)
        self.psi_h = np.zeros(n_bfs)
        
        # 正则化系统
        self.cs = CanonicalSystem(dt=self.dt, **kwargs)
        self.timesteps = round(self.cs.run_time / self.dt)
        
        # 生成基函数中心
        self.generate_centers()
        
        # 计算基函数宽度
        self.h = np.ones(self.n_bfs) * self.n_bfs**1.5 / self.psi_centers / self.cs.alpha_x
        
        # 重置状态
        self.reset_state()
    
    def generate_centers(self):
        """生成高斯基函数的中心"""
        t_centers = np.linspace(0, self.cs.run_time, self.n_bfs)
        
        cs = self.cs
        x_track = cs.run()
        t_track = np.linspace(0, cs.run_time, cs.timesteps)
        
        for n in range(len(t_centers)):
            for i, t in enumerate(t_track):
                if abs(t_centers[n] - t) <= cs.dt:
                    self.psi_centers[n] = x_track[i]
        
        return self.psi_centers
    
    def generate_psi(self, x):
        """生成基函数激活"""
        if isinstance(x, np.ndarray):
            x = x[:, None]
        
        self.psi = np.exp(-self.h * (x - self.psi_centers)**2)
        return self.psi
    
    def learning_from_gmr(self, gmr_trajectory, timestamps=None):
        """
        从GMR回归轨迹学习DMP参数
        
        Args:
            gmr_trajectory: GMR回归的轨迹，形状为 (n_timesteps, n_dims)
            timestamps: 时间戳
        """
        self.gmr_trajectory = np.array(gmr_trajectory)
        
        if timestamps is None:
            timestamps = np.linspace(0, 1, len(gmr_trajectory))
        
        # 设置起始和目标位置
        self.y0 = gmr_trajectory[0].copy()
        self.goal = gmr_trajectory[-1].copy()
        
        # 插值到DMP的时间步长
        n_timesteps = self.timesteps
        interpolated_trajectory = np.zeros((n_timesteps, self.n_dmps))
        
        for d in range(self.n_dmps):
            interp_func = interp1d(timestamps, gmr_trajectory[:, d], 
                                 kind='cubic', fill_value='extrapolate')
            interpolated_trajectory[:, d] = interp_func(np.linspace(0, 1, n_timesteps))
        
        # 计算速度和加速度
        dy_demo = np.gradient(interpolated_trajectory, axis=0) / self.dt
        ddy_demo = np.gradient(dy_demo, axis=0) / self.dt
        
        # 计算目标强制项
        x_track = self.cs.run()
        f_target = np.zeros((n_timesteps, self.n_dmps))
        
        for d in range(self.n_dmps):
            k = self.alpha_y[d]
            f_target[:, d] = (ddy_demo[:, d] - self.alpha_y[d] * 
                            (self.beta_y[d] * (self.goal[d] - interpolated_trajectory[:, d]) - 
                             dy_demo[:, d])) / k + x_track * (self.goal[d] - self.y0[d])
        
        # 学习权重
        self.generate_weights(f_target)
        
        print("从GMR轨迹学习DMP参数完成")
    
    def generate_weights(self, f_target):
        """生成强制项权重"""
        x_track = self.cs.run()
        psi_track = self.generate_psi(x_track)
        
        for d in range(self.n_dmps):
            delta = self.goal[d] - self.y0[d]
            
            for b in range(self.n_bfs):
                numer = np.sum(x_track * psi_track[:, b] * f_target[:, d])
                denom = np.sum(x_track**2 * psi_track[:, b])
                
                self.w[d, b] = numer / denom
                if abs(delta) > 1e-6:
                    self.w[d, b] = self.w[d, b] / delta
        
        self.w = np.nan_to_num(self.w)
        return self.w
    
    def reset_state(self):
        """重置系统状态"""
        self.y = self.y0.copy()
        self.dy = np.zeros(self.n_dmps)
        self.ddy = np.zeros(self.n_dmps)
        self.cs.reset_state()
    
    def step(self, tau=None):
        """执行一步DMP"""
        if tau is None:
            tau = self.tau
        
        # 运行正则化系统
        x = self.cs.step_discrete(tau)
        
        # 生成基函数激活
        psi = self.generate_psi(x)
        
        for d in range(self.n_dmps):
            # 生成强制项
            k = self.alpha_y[d]
            delta = self.goal[d] - self.y0[d]
            
            if abs(delta) > 1e-5:
                k2 = delta / delta
            else:
                k2 = 1.0
            
            f = k * (np.dot(psi, self.w[d]) * x * k2 / np.sum(psi)) - k * (self.goal[d] - self.y0[d]) * x
            
            # 生成复现轨迹
            self.ddy[d] = self.alpha_y[d] * (self.beta_y[d] * (self.goal[d] - self.y[d]) - self.dy[d]) + f
            self.dy[d] += tau * self.ddy[d] * self.dt
            self.y[d] += tau * self.dy[d] * self.dt
        
        return self.y, self.dy, self.ddy
    
    def reproduce(self, tau=None, initial=None, goal=None):
        """
        复现轨迹
        
        Args:
            tau: 时间缩放因子
            initial: 初始位置
            goal: 目标位置
            
        Returns:
            reproduced_trajectory: 复现的轨迹
            velocity: 速度
            acceleration: 加速度
        """
        # 设置时间缩放
        if tau is None:
            timesteps = self.timesteps
        else:
            timesteps = round(self.timesteps / tau)
        
        # 设置初始状态
        if initial is not None:
            self.y0 = np.array(initial)
        
        # 设置目标状态
        if goal is not None:
            self.goal = np.array(goal)
        
        # 重置状态
        self.reset_state()
        
        # 复现轨迹
        reproduced_trajectory = np.zeros((timesteps, self.n_dmps))
        velocity = np.zeros((timesteps, self.n_dmps))
        acceleration = np.zeros((timesteps, self.n_dmps))
        
        for t in range(timesteps):
            reproduced_trajectory[t], velocity[t], acceleration[t] = self.step(tau=tau)
        
        return reproduced_trajectory, velocity, acceleration
    
    def generalize(self, new_initial=None, new_goal=None, tau=1.0):
        """
        轨迹泛化
        
        Args:
            new_initial: 新的初始位置
            new_goal: 新的目标位置
            tau: 时间缩放因子
            
        Returns:
            generalized_trajectory: 泛化后的轨迹
        """
        if new_initial is None:
            new_initial = self.y0
        if new_goal is None:
            new_goal = self.goal
        
        generalized_trajectory, _, _ = self.reproduce(tau=tau, initial=new_initial, goal=new_goal)
        
        return generalized_trajectory
    
    def visualize_comparison(self, gmr_trajectory=None, reproduced_trajectory=None, 
                           generalized_trajectory=None, dim1=0, dim2=1, save_path=None):
        """
        可视化比较结果
        
        Args:
            gmr_trajectory: GMR回归轨迹
            reproduced_trajectory: DMP复现轨迹
            generalized_trajectory: DMP泛化轨迹
            dim1: 第一个维度索引
            dim2: 第二个维度索引
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=(12, 8))
        
        if gmr_trajectory is not None:
            ax.plot(gmr_trajectory[:, dim1], gmr_trajectory[:, dim2], 
                   'b-', linewidth=2, label='GMR Regression')
        
        if reproduced_trajectory is not None:
            ax.plot(reproduced_trajectory[:, dim1], reproduced_trajectory[:, dim2], 
                   'r--', linewidth=2, label='DMP Reproduction')
        
        if generalized_trajectory is not None:
            ax.plot(generalized_trajectory[:, dim1], generalized_trajectory[:, dim2], 
                   'g:', linewidth=2, label='DMP Generalization')
        
        ax.set_xlabel(f'Dimension {dim1}')
        ax.set_ylabel(f'Dimension {dim2}')
        ax.set_title('DMP Trajectory Comparison')
        ax.legend()
        ax.grid(True)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"DMP比较可视化图保存到: {save_path}")
        
        plt.show()
    
    def visualize_3d_comparison(self, gmr_trajectory=None, reproduced_trajectory=None, 
                              generalized_trajectory=None, save_path=None):
        """
        3D比较可视化
        
        Args:
            gmr_trajectory: GMR回归轨迹
            reproduced_trajectory: DMP复现轨迹
            generalized_trajectory: DMP泛化轨迹
            save_path: 保存路径
        """
        if self.n_dmps < 3:
            print("轨迹维度小于3，无法进行3D可视化")
            return
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        if gmr_trajectory is not None:
            ax.plot(gmr_trajectory[:, 0], gmr_trajectory[:, 1], gmr_trajectory[:, 2], 
                   'b-', linewidth=2, label='GMR Regression')
        
        if reproduced_trajectory is not None:
            ax.plot(reproduced_trajectory[:, 0], reproduced_trajectory[:, 1], reproduced_trajectory[:, 2], 
                   'r--', linewidth=2, label='DMP Reproduction')
        
        if generalized_trajectory is not None:
            ax.plot(generalized_trajectory[:, 0], generalized_trajectory[:, 1], generalized_trajectory[:, 2], 
                   'g:', linewidth=2, label='DMP Generalization')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D DMP Trajectory Comparison')
        ax.legend()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"3D DMP比较可视化图保存到: {save_path}")
        
        plt.show()

# 测试代码
if __name__ == "__main__":
    # 生成测试GMR轨迹
    np.random.seed(42)
    n_timesteps = 100
    n_dims = 3
    
    t = np.linspace(0, 2*np.pi, n_timesteps)
    gmr_trajectory = np.column_stack([
        np.sin(t),
        np.cos(t),
        t/(2*np.pi)
    ])
    
    # 创建改进的DMP
    improved_dmp = ImprovedDMP(n_dmps=n_dims, n_bfs=50, dt=0.01)
    
    # 从GMR轨迹学习
    improved_dmp.learning_from_gmr(gmr_trajectory)
    
    # 复现轨迹
    reproduced_trajectory, _, _ = improved_dmp.reproduce()
    
    # 泛化轨迹
    new_initial = [0.5, 0.5, 0.2]
    new_goal = [-0.5, -0.5, 0.8]
    generalized_trajectory = improved_dmp.generalize(new_initial, new_goal)
    
    # 可视化
    improved_dmp.visualize_comparison(gmr_trajectory, reproduced_trajectory, generalized_trajectory)
    improved_dmp.visualize_3d_comparison(gmr_trajectory, reproduced_trajectory, generalized_trajectory)
    
    print(f"GMR轨迹形状: {gmr_trajectory.shape}")
    print(f"复现轨迹形状: {reproduced_trajectory.shape}")
    print(f"泛化轨迹形状: {generalized_trajectory.shape}")
