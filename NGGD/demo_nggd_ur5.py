# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - UR5机器人演示程序
结合CoppeliaSim仿真环境展示NGGD算法的实际应用
"""

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# 添加路径
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)
sys.path.append(os.path.join(current_dir, '..'))

# CoppeliaSim相关导入
sys.path.append('C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/legacyRemoteApi/remoteApiBindings/python/python')
import sim as vrep_sim

# UR5相关导入
sys.path.append('C:/Users/19046/Desktop/UR5Test/PyDMPs/code/UR5')
sys.path.append('C:/Users/19046/Desktop/UR5Test/PyDMPs/code/UR5/VREP_RemoteAPIs')
from UR5SimModel import UR5SimModel

# NGGD算法导入
from nggd_imitation_learning import NGGDImitationLearning

class NGGDUR5Demo:
    """NGGD算法在UR5机器人上的演示类"""
    
    def __init__(self):
        """初始化演示系统"""
        self.client_ID = None
        self.ur5_sim_model = None
        self.nggd_system = None
        self.demonstration_trajectories = []
        
    def connect_to_coppelia(self):
        """连接到CoppeliaSim"""
        print("连接到CoppeliaSim...")
        vrep_sim.simxFinish(-1)
        
        while True:
            self.client_ID = vrep_sim.simxStart('127.0.0.1', 19999, True, False, 5000, 5)
            if self.client_ID > -1:
                print('成功连接到CoppeliaSim')
                break
            else:
                print('连接失败，重试中...')
                time.sleep(1)
        
        # 设置仿真参数
        delta_t = 0.01
        vrep_sim.simxSetFloatingParameter(self.client_ID, vrep_sim.sim_floatparam_simulation_time_step, 
                                        delta_t, vrep_sim.simx_opmode_oneshot)
        vrep_sim.simxSynchronous(self.client_ID, True)
        vrep_sim.simxStartSimulation(self.client_ID, vrep_sim.simx_opmode_oneshot)
        
        # 初始化UR5模型
        self.ur5_sim_model = UR5SimModel()
        self.ur5_sim_model.initializeSimModel(self.client_ID)
        
        print("CoppeliaSim连接和UR5模型初始化完成")
    
    def record_demonstration_trajectory(self, trajectory_name="demo_trajectory"):
        """
        记录示教轨迹
        
        Args:
            trajectory_name: 轨迹名称
        """
        print(f"开始记录示教轨迹: {trajectory_name}")
        print("请手动移动UR5机器人末端执行器...")
        
        # 获取dummy句柄
        return_code, via_dummy_handle = vrep_sim.simxGetObjectHandle(
            self.client_ID, 'via', vrep_sim.simx_opmode_blocking)
        
        if return_code != vrep_sim.simx_return_ok:
            print("无法获取via dummy句柄")
            return None
        
        # 记录轨迹
        trajectory = []
        recording_time = 10.0  # 记录10秒
        start_time = time.time()
        
        while time.time() - start_time < recording_time:
            # 获取当前位置
            return_code, position = vrep_sim.simxGetObjectPosition(
                self.client_ID, via_dummy_handle, -1, vrep_sim.simx_opmode_buffer)
            
            if return_code == vrep_sim.simx_return_ok:
                trajectory.append(position)
            
            vrep_sim.simxSynchronousTrigger(self.client_ID)
            time.sleep(0.01)
        
        trajectory = np.array(trajectory)
        print(f"轨迹记录完成，共{len(trajectory)}个点")
        
        # 保存轨迹
        trajectory_file = os.path.join(current_dir, f"{trajectory_name}.csv")
        pd.DataFrame(trajectory).to_csv(trajectory_file, header=False, index=False)
        print(f"轨迹保存到: {trajectory_file}")
        
        return trajectory
    
    def load_demonstration_trajectories(self, trajectory_files):
        """
        加载示教轨迹文件
        
        Args:
            trajectory_files: 轨迹文件路径列表
        """
        print("加载示教轨迹文件...")
        
        for i, file_path in enumerate(trajectory_files):
            if os.path.exists(file_path):
                df = pd.read_csv(file_path, header=None)
                trajectory = np.array(df)
                self.demonstration_trajectories.append(trajectory)
                print(f"加载轨迹 {i+1}: {file_path}, 形状: {trajectory.shape}")
            else:
                print(f"文件不存在: {file_path}")
        
        print(f"共加载 {len(self.demonstration_trajectories)} 条示教轨迹")
    
    def generate_synthetic_demonstrations(self, n_demos=6):
        """
        生成合成示教数据（用于测试）
        
        Args:
            n_demos: 示教数量
        """
        print(f"生成 {n_demos} 条合成示教轨迹...")
        
        np.random.seed(42)
        n_timesteps = 100
        n_dims = 3
        
        for i in range(n_demos):
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
            
            self.demonstration_trajectories.append(trajectory)
        
        print(f"生成了 {len(self.demonstration_trajectories)} 条合成示教轨迹")
    
    def learn_with_nggd(self, n_gmm_components=10, n_dmp_bfs=100, noise_std=0.01):
        """
        使用NGGD算法学习
        
        Args:
            n_gmm_components: GMM组件数量
            n_dmp_bfs: DMP基函数数量
            noise_std: 噪声标准差
        """
        if not self.demonstration_trajectories:
            print("没有示教轨迹数据，请先加载或生成示教轨迹")
            return
        
        print("开始NGGD模仿学习...")
        
        # 创建NGGD系统
        self.nggd_system = NGGDImitationLearning(
            n_gmm_components=n_gmm_components,
            n_dmp_bfs=n_dmp_bfs,
            noise_std=noise_std
        )
        
        # 执行学习
        learning_results = self.nggd_system.learn_from_demonstrations(
            self.demonstration_trajectories,
            alignment_method='linear',
            add_noise=True,
            normalize=True,
            plot_results=True
        )
        
        print("NGGD学习完成!")
        return learning_results
    
    def execute_trajectory_in_simulation(self, trajectory, trajectory_name="execution"):
        """
        在仿真中执行轨迹
        
        Args:
            trajectory: 要执行的轨迹
            trajectory_name: 轨迹名称
        """
        print(f"在仿真中执行轨迹: {trajectory_name}")
        
        # 获取dummy句柄
        return_code, via_dummy_handle = vrep_sim.simxGetObjectHandle(
            self.client_ID, 'via', vrep_sim.simx_opmode_blocking)
        
        if return_code != vrep_sim.simx_return_ok:
            print("无法获取via dummy句柄")
            return
        
        # 执行轨迹
        for i, position in enumerate(trajectory):
            vrep_sim.simxSetObjectPosition(self.client_ID, via_dummy_handle, -1, 
                                         position, vrep_sim.simx_opmode_oneshot)
            vrep_sim.simxSynchronousTrigger(self.client_ID)
            time.sleep(0.01)
        
        print(f"轨迹执行完成，共{len(trajectory)}个点")
    
    def demonstrate_generalization(self):
        """演示轨迹泛化能力"""
        if self.nggd_system is None:
            print("请先执行NGGD学习")
            return
        
        print("\n演示轨迹泛化能力...")
        
        # 定义不同的泛化场景
        generalization_scenarios = [
            {
                'name': '场景1: 不同初始和目标位置',
                'initial': [0.3, 0.3, 0.2],
                'goal': [-0.3, -0.3, 0.8],
                'tau': 1.0
            },
            {
                'name': '场景2: 时间缩放',
                'initial': [0.2, 0.8, 0.1],
                'goal': [0.8, 0.2, 0.9],
                'tau': 0.5
            },
            {
                'name': '场景3: 另一个轨迹',
                'initial': [-0.4, 0.6, 0.3],
                'goal': [0.6, -0.4, 0.7],
                'tau': 1.2
            }
        ]
        
        for i, scenario in enumerate(generalization_scenarios):
            print(f"\n{scenario['name']}")
            print("-" * 40)
            
            # 生成泛化轨迹
            generalized_trajectory = self.nggd_system.generalize_trajectory(
                new_initial=scenario['initial'],
                new_goal=scenario['goal'],
                tau=scenario['tau'],
                plot_comparison=True
            )
            
            # 在仿真中执行（可选）
            user_input = input(f"是否在仿真中执行场景{i+1}? (y/n): ")
            if user_input.lower() == 'y':
                self.execute_trajectory_in_simulation(
                    generalized_trajectory, 
                    f"generalization_scenario_{i+1}"
                )
    
    def visualize_all_trajectories(self):
        """可视化所有轨迹"""
        if not self.demonstration_trajectories:
            print("没有轨迹数据可视化")
            return
        
        fig = plt.figure(figsize=(15, 10))
        
        # 2D可视化
        ax1 = fig.add_subplot(221)
        for i, traj in enumerate(self.demonstration_trajectories):
            ax1.plot(traj[:, 0], traj[:, 1], alpha=0.7, linewidth=1, 
                    label=f'Demo {i+1}' if i < 5 else "")
        ax1.set_title('Demonstration Trajectories (2D)')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.legend()
        ax1.grid(True)
        
        # 3D可视化
        ax2 = fig.add_subplot(222, projection='3d')
        for i, traj in enumerate(self.demonstration_trajectories):
            ax2.plot(traj[:, 0], traj[:, 1], traj[:, 2], alpha=0.7, linewidth=1)
        ax2.set_title('Demonstration Trajectories (3D)')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        
        # 如果有NGGD结果，也显示
        if self.nggd_system is not None:
            ax3 = fig.add_subplot(223)
            for i, traj in enumerate(self.demonstration_trajectories):
                ax3.plot(traj[:, 0], traj[:, 1], alpha=0.3, linewidth=1, color='blue')
            ax3.plot(self.nggd_system.gmr_trajectory[:, 0], 
                    self.nggd_system.gmr_trajectory[:, 1], 
                    'g-', linewidth=2, label='GMR')
            ax3.plot(self.nggd_system.reproduced_trajectory[:, 0], 
                    self.nggd_system.reproduced_trajectory[:, 1], 
                    'r--', linewidth=2, label='DMP')
            ax3.set_title('NGGD Results')
            ax3.set_xlabel('X')
            ax3.set_ylabel('Y')
            ax3.legend()
            ax3.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def run_complete_demo(self):
        """运行完整演示"""
        print("=" * 60)
        print("NGGD模仿学习算法 - UR5机器人演示")
        print("=" * 60)
        
        try:
            # 1. 连接仿真环境
            self.connect_to_coppelia()
            
            # 2. 生成或加载示教数据
            demo_choice = input("选择演示模式:\n1. 使用合成数据\n2. 记录真实示教\n3. 加载已有轨迹\n请输入选择 (1/2/3): ")
            
            if demo_choice == '1':
                self.generate_synthetic_demonstrations(n_demos=6)
            elif demo_choice == '2':
                n_demos = int(input("请输入要记录的示教数量: "))
                for i in range(n_demos):
                    input(f"按回车开始记录第{i+1}条示教轨迹...")
                    traj = self.record_demonstration_trajectory(f"demo_{i+1}")
                    if traj is not None:
                        self.demonstration_trajectories.append(traj)
            elif demo_choice == '3':
                # 加载现有轨迹文件
                trajectory_files = [
                    'C:/Users/19046/Desktop/UR5Test/PyDMPs/code/demo_trajectory/demo_trajectory_for_discrete_dmp.csv'
                ]
                self.load_demonstration_trajectories(trajectory_files)
            
            # 3. 可视化示教轨迹
            self.visualize_all_trajectories()
            
            # 4. NGGD学习
            self.learn_with_nggd(n_gmm_components=10, n_dmp_bfs=80, noise_std=0.02)
            
            # 5. 演示泛化能力
            self.demonstrate_generalization()
            
            print("\n演示完成!")
            
        except KeyboardInterrupt:
            print("\n演示被用户中断")
        except Exception as e:
            print(f"演示过程中出现错误: {e}")
        finally:
            # 清理资源
            if self.client_ID is not None:
                vrep_sim.simxStopSimulation(self.client_ID, vrep_sim.simx_opmode_blocking)
                vrep_sim.simxFinish(-1)
                print("仿真环境已关闭")

# 主程序
if __name__ == "__main__":
    demo = NGGDUR5Demo()
    demo.run_complete_demo()
