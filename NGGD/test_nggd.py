# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - 测试脚本
验证NGGD算法的基本功能
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 添加模块路径
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)

from nggd_imitation_learning import NGGDImitationLearning

def generate_test_trajectories():
    """生成测试轨迹数据"""
    print("生成测试轨迹数据...")
    
    np.random.seed(42)
    n_demonstrations = 6
    n_timesteps = 100
    n_dims = 3
    
    trajectories = []
    
    for i in range(n_demonstrations):
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
        
        trajectories.append(trajectory)
    
    return trajectories

def test_nggd_algorithm():
    """测试NGGD算法"""
    print("=" * 60)
    print("NGGD模仿学习算法测试")
    print("=" * 60)
    
    # 1. 生成测试数据
    trajectories = generate_test_trajectories()
    print(f"生成了 {len(trajectories)} 条测试轨迹")
    
    # 2. 创建NGGD系统
    print("\n创建NGGD系统...")
    nggd_system = NGGDImitationLearning(
        n_gmm_components=8,
        n_dmp_bfs=60,
        noise_std=0.02
    )
    
    # 3. 执行学习
    print("\n执行NGGD学习...")
    try:
        learning_results = nggd_system.learn_from_demonstrations(
            trajectories,
            alignment_method='linear',
            add_noise=True,
            normalize=True,
            plot_results=True
        )
        print("✓ NGGD学习成功完成")
    except Exception as e:
        print(f"✗ NGGD学习失败: {e}")
        return False
    
    # 4. 测试轨迹泛化
    print("\n测试轨迹泛化...")
    try:
        # 泛化测试1
        generalized_traj1 = nggd_system.generalize_trajectory(
            new_initial=[0.3, 0.3, 0.2],
            new_goal=[-0.3, -0.3, 0.8],
            tau=1.0,
            plot_comparison=True
        )
        print("✓ 泛化测试1成功")
        
        # 泛化测试2
        generalized_traj2 = nggd_system.generalize_trajectory(
            new_initial=[0.2, 0.8, 0.1],
            new_goal=[0.8, 0.2, 0.9],
            tau=0.5,
            plot_comparison=True
        )
        print("✓ 泛化测试2成功")
        
        # 泛化测试3
        generalized_traj3 = nggd_system.generalize_trajectory(
            new_initial=[-0.4, 0.6, 0.3],
            new_goal=[0.6, -0.4, 0.7],
            tau=1.2,
            plot_comparison=True
        )
        print("✓ 泛化测试3成功")
        
    except Exception as e:
        print(f"✗ 轨迹泛化失败: {e}")
        return False
    
    # 5. 验证结果
    print("\n验证学习结果...")
    
    # 检查GMR轨迹
    gmr_trajectory = learning_results['gmr_trajectory']
    print(f"GMR轨迹形状: {gmr_trajectory.shape}")
    
    # 检查DMP复现轨迹
    reproduced_trajectory = learning_results['reproduced_trajectory']
    print(f"DMP复现轨迹形状: {reproduced_trajectory.shape}")
    
    # 检查泛化轨迹
    print(f"泛化轨迹1形状: {generalized_traj1.shape}")
    print(f"泛化轨迹2形状: {generalized_traj2.shape}")
    print(f"泛化轨迹3形状: {generalized_traj3.shape}")
    
    # 6. 计算性能指标
    print("\n计算性能指标...")
    
    # 计算GMR和DMP复现的相似度
    from scipy.spatial.distance import cdist
    gmr_dmp_distance = np.mean(cdist(gmr_trajectory, reproduced_trajectory))
    print(f"GMR与DMP复现的平均距离: {gmr_dmp_distance:.4f}")
    
    # 计算轨迹平滑度（通过加速度变化）
    reproduced_acc = np.gradient(np.gradient(reproduced_trajectory, axis=0), axis=0)
    smoothness = np.mean(np.linalg.norm(reproduced_acc, axis=1))
    print(f"DMP复现轨迹平滑度: {smoothness:.4f}")
    
    print("\n" + "=" * 60)
    print("NGGD算法测试完成!")
    print("=" * 60)
    
    return True

def test_individual_modules():
    """测试各个模块"""
    print("\n测试各个模块...")
    
    # 测试数据预处理
    try:
        from data_preprocessing import TrajectoryPreprocessor
        preprocessor = TrajectoryPreprocessor()
        print("✓ 数据预处理模块导入成功")
    except Exception as e:
        print(f"✗ 数据预处理模块导入失败: {e}")
    
    # 测试GMM建模
    try:
        from gmm_trajectory_modeling import GMMTrajectoryModeling
        gmm_modeler = GMMTrajectoryModeling()
        print("✓ GMM建模模块导入成功")
    except Exception as e:
        print(f"✗ GMM建模模块导入失败: {e}")
    
    # 测试GMR回归
    try:
        from gmr_trajectory_regression import GMRTrajectoryRegression
        print("✓ GMR回归模块导入成功")
    except Exception as e:
        print(f"✗ GMR回归模块导入失败: {e}")
    
    # 测试改进DMP
    try:
        from improved_dmp import ImprovedDMP
        improved_dmp = ImprovedDMP()
        print("✓ 改进DMP模块导入成功")
    except Exception as e:
        print(f"✗ 改进DMP模块导入失败: {e}")

if __name__ == "__main__":
    # 测试各个模块
    test_individual_modules()
    
    # 测试完整算法
    success = test_nggd_algorithm()
    
    if success:
        print("\n🎉 所有测试通过！NGGD算法工作正常。")
    else:
        print("\n❌ 测试失败，请检查错误信息。")
