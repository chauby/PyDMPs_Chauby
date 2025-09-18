# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - 使用示例
展示如何使用NGGD算法进行模仿学习
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 添加模块路径
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)

from nggd_imitation_learning import NGGDImitationLearning

def create_demonstration_data():
    """创建示教数据示例"""
    print("创建示教数据...")
    
    np.random.seed(42)
    n_demonstrations = 5
    n_timesteps = 100
    n_dims = 3
    
    trajectories = []
    
    # 创建不同类型的示教轨迹
    for i in range(n_demonstrations):
        t = np.linspace(0, 2*np.pi, n_timesteps)
        
        if i == 0:  # 圆形轨迹
            trajectory = np.column_stack([
                np.sin(t),
                np.cos(t),
                t/(2*np.pi)
            ])
        elif i == 1:  # 8字形轨迹
            trajectory = np.column_stack([
                np.sin(t),
                np.sin(2*t),
                t/(2*np.pi)
            ])
        elif i == 2:  # 螺旋轨迹
            trajectory = np.column_stack([
                t/(2*np.pi) * np.sin(t),
                t/(2*np.pi) * np.cos(t),
                t/(2*np.pi)
            ])
        elif i == 3:  # 直线轨迹
            trajectory = np.column_stack([
                np.linspace(-1, 1, n_timesteps),
                np.zeros(n_timesteps),
                t/(2*np.pi)
            ])
        else:  # 复杂轨迹
            trajectory = np.column_stack([
                np.sin(t) * np.cos(2*t),
                np.cos(t) * np.sin(2*t),
                t/(2*np.pi)
            ])
        
        # 添加一些噪声
        noise = np.random.normal(0, 0.05, trajectory.shape)
        trajectory += noise
        
        trajectories.append(trajectory)
    
    return trajectories

def example_basic_usage():
    """基本使用示例"""
    print("=" * 60)
    print("NGGD模仿学习 - 基本使用示例")
    print("=" * 60)
    
    # 1. 创建示教数据
    demonstration_trajectories = create_demonstration_data()
    print(f"创建了 {len(demonstration_trajectories)} 条示教轨迹")
    
    # 2. 创建NGGD系统
    nggd_system = NGGDImitationLearning(
        n_gmm_components=8,    # GMM组件数量
        n_dmp_bfs=60,          # DMP基函数数量
        noise_std=0.02         # 噪声标准差
    )
    
    # 3. 执行学习
    print("\n执行NGGD学习...")
    learning_results = nggd_system.learn_from_demonstrations(
        demonstration_trajectories,
        alignment_method='linear',
        add_noise=True,
        normalize=True,
        plot_results=True
    )
    
    # 4. 轨迹泛化示例
    print("\n轨迹泛化示例...")
    
    # 示例1: 不同的初始和目标位置
    print("示例1: 不同初始和目标位置")
    generalized_traj1 = nggd_system.generalize_trajectory(
        new_initial=[0.5, 0.5, 0.2],
        new_goal=[-0.5, -0.5, 0.8],
        tau=1.0,
        plot_comparison=True
    )
    
    # 示例2: 时间缩放
    print("示例2: 时间缩放")
    generalized_traj2 = nggd_system.generalize_trajectory(
        new_initial=[0.2, 0.8, 0.1],
        new_goal=[0.8, 0.2, 0.9],
        tau=0.5,
        plot_comparison=True
    )
    
    # 示例3: 另一个轨迹
    print("示例3: 另一个轨迹")
    generalized_traj3 = nggd_system.generalize_trajectory(
        new_initial=[-0.3, 0.7, 0.3],
        new_goal=[0.7, -0.3, 0.7],
        tau=1.2,
        plot_comparison=True
    )
    
    return learning_results, [generalized_traj1, generalized_traj2, generalized_traj3]

def example_advanced_usage():
    """高级使用示例"""
    print("\n" + "=" * 60)
    print("NGGD模仿学习 - 高级使用示例")
    print("=" * 60)
    
    # 创建更复杂的示教数据
    np.random.seed(123)
    n_demonstrations = 8
    n_timesteps = 150
    n_dims = 3
    
    trajectories = []
    
    for i in range(n_demonstrations):
        t = np.linspace(0, 3*np.pi, n_timesteps)
        
        # 创建更复杂的轨迹模式
        if i % 4 == 0:  # 圆形
            trajectory = np.column_stack([
                np.sin(t) + 0.1 * np.sin(3*t),
                np.cos(t) + 0.1 * np.cos(3*t),
                t/(3*np.pi)
            ])
        elif i % 4 == 1:  # 8字形
            trajectory = np.column_stack([
                np.sin(t) + 0.2 * np.sin(4*t),
                np.sin(2*t) + 0.2 * np.sin(5*t),
                t/(3*np.pi)
            ])
        elif i % 4 == 2:  # 螺旋
            trajectory = np.column_stack([
                t/(3*np.pi) * np.sin(t) + 0.1 * np.sin(2*t),
                t/(3*np.pi) * np.cos(t) + 0.1 * np.cos(2*t),
                t/(3*np.pi)
            ])
        else:  # 复杂轨迹
            trajectory = np.column_stack([
                np.sin(t) * np.cos(2*t) + 0.1 * np.sin(5*t),
                np.cos(t) * np.sin(2*t) + 0.1 * np.cos(5*t),
                t/(3*np.pi)
            ])
        
        # 添加噪声
        noise = np.random.normal(0, 0.03, trajectory.shape)
        trajectory += noise
        
        trajectories.append(trajectory)
    
    # 使用更多GMM组件和DMP基函数
    nggd_system = NGGDImitationLearning(
        n_gmm_components=15,
        n_dmp_bfs=100,
        noise_std=0.01
    )
    
    # 执行学习
    learning_results = nggd_system.learn_from_demonstrations(
        trajectories,
        alignment_method='linear',
        add_noise=True,
        normalize=True,
        plot_results=True
    )
    
    # 测试多个泛化场景
    generalization_scenarios = [
        {"name": "场景1", "initial": [0.4, 0.4, 0.1], "goal": [-0.4, -0.4, 0.9], "tau": 1.0},
        {"name": "场景2", "initial": [0.1, 0.9, 0.2], "goal": [0.9, 0.1, 0.8], "tau": 0.7},
        {"name": "场景3", "initial": [-0.5, 0.5, 0.3], "goal": [0.5, -0.5, 0.7], "tau": 1.3},
        {"name": "场景4", "initial": [0.0, 0.0, 0.0], "goal": [1.0, 1.0, 1.0], "tau": 0.8}
    ]
    
    generalized_trajectories = []
    for scenario in generalization_scenarios:
        print(f"\n{scenario['name']}: 初始={scenario['initial']}, 目标={scenario['goal']}, tau={scenario['tau']}")
        
        generalized_traj = nggd_system.generalize_trajectory(
            new_initial=scenario['initial'],
            new_goal=scenario['goal'],
            tau=scenario['tau'],
            plot_comparison=True
        )
        
        generalized_trajectories.append(generalized_traj)
    
    return learning_results, generalized_trajectories

def visualize_all_results(learning_results, generalized_trajectories):
    """可视化所有结果"""
    print("\n生成综合可视化...")
    
    fig = plt.figure(figsize=(20, 12))
    
    # 原始示教轨迹
    ax1 = fig.add_subplot(2, 3, 1)
    for i, traj in enumerate(learning_results['processed_trajectories']):
        ax1.plot(traj[:, 0], traj[:, 1], alpha=0.6, linewidth=1, 
                label=f'Demo {i+1}' if i < 3 else "")
    ax1.set_title('Demonstration Trajectories')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.legend()
    ax1.grid(True)
    
    # GMR回归结果
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(learning_results['gmr_trajectory'][:, 0], 
            learning_results['gmr_trajectory'][:, 1], 
            'g-', linewidth=2, label='GMR Regression')
    ax2.set_title('GMR Regression')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.legend()
    ax2.grid(True)
    
    # DMP复现结果
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(learning_results['gmr_trajectory'][:, 0], 
            learning_results['gmr_trajectory'][:, 1], 
            'g-', linewidth=2, label='GMR Reference')
    ax3.plot(learning_results['reproduced_trajectory'][:, 0], 
            learning_results['reproduced_trajectory'][:, 1], 
            'r--', linewidth=2, label='DMP Reproduction')
    ax3.set_title('DMP Reproduction')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.legend()
    ax3.grid(True)
    
    # 泛化结果
    ax4 = fig.add_subplot(2, 3, 4)
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    for i, traj in enumerate(generalized_trajectories):
        ax4.plot(traj[:, 0], traj[:, 1], 
                color=colors[i % len(colors)], linewidth=2, 
                label=f'Generalization {i+1}')
    ax4.set_title('Generalization Results')
    ax4.set_xlabel('X')
    ax4.set_ylabel('Y')
    ax4.legend()
    ax4.grid(True)
    
    # 3D可视化
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    for i, traj in enumerate(learning_results['processed_trajectories'][:3]):
        ax5.plot(traj[:, 0], traj[:, 1], traj[:, 2], alpha=0.6, linewidth=1)
    ax5.plot(learning_results['gmr_trajectory'][:, 0], 
            learning_results['gmr_trajectory'][:, 1], 
            learning_results['gmr_trajectory'][:, 2], 
            'g-', linewidth=2, label='GMR')
    ax5.plot(learning_results['reproduced_trajectory'][:, 0], 
            learning_results['reproduced_trajectory'][:, 1], 
            learning_results['reproduced_trajectory'][:, 2], 
            'r--', linewidth=2, label='DMP')
    ax5.set_title('3D Visualization')
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_zlabel('Z')
    ax5.legend()
    
    # 轨迹长度比较
    ax6 = fig.add_subplot(2, 3, 6)
    lengths = [len(traj) for traj in learning_results['processed_trajectories']]
    ax6.bar(range(len(lengths)), lengths, alpha=0.7)
    ax6.set_title('Trajectory Lengths')
    ax6.set_xlabel('Trajectory Index')
    ax6.set_ylabel('Length (timesteps)')
    ax6.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    """主函数"""
    print("NGGD模仿学习算法 - 使用示例")
    print("=" * 60)
    
    try:
        # 基本使用示例
        learning_results, generalized_trajectories = example_basic_usage()
        
        # 高级使用示例
        advanced_results, advanced_generalized = example_advanced_usage()
        
        # 综合可视化
        visualize_all_results(learning_results, generalized_trajectories)
        
        print("\n🎉 所有示例运行完成！")
        print("\nNGGD算法特点:")
        print("1. 能够学习复杂的示教轨迹模式")
        print("2. 通过GMM建模捕获轨迹的概率分布")
        print("3. 使用GMR进行概率最优的轨迹回归")
        print("4. 基于DMP实现灵活的轨迹泛化")
        print("5. 支持不同的初始位置、目标位置和时间缩放")
        
    except Exception as e:
        print(f"❌ 运行示例时出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
