# -*- coding: utf-8 -*-
#%% import package
import time
import numpy as np
import pandas as pd
import os
from datetime import datetime

import sys
sys.path.append('./UR5/VREP_RemoteAPIs')
import sim as vrep_sim

sys.path.append('./UR5')
from UR5SimModel import UR5SimModel

sys.path.append('./DMP')
from dmp_discrete import dmp_discrete

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#%% program
print ('Program started')

# ------------------------------- Connect to VREP (CoppeliaSim) ------------------------------- 
vrep_sim.simxFinish(-1) # just in case, close all opened connections
while True:
    client_ID = vrep_sim.simxStart('127.0.0.1', 19999, True, False, 5000, 5) # Connect to CoppeliaSim
    if client_ID > -1: # connected
        print('Connect to remote API server.')
        break
    else:
        print('Failed connecting to remote API server! Try it again ...')

# Pause the simulation
# res = vrep_sim.simxPauseSimulation(client_ID, vrep_sim.simx_opmode_blocking)

delta_t = 0.01 # simulation time step
# Set the simulation step size for VREP
vrep_sim.simxSetFloatingParameter(client_ID, vrep_sim.sim_floatparam_simulation_time_step, delta_t, vrep_sim.simx_opmode_oneshot)
# Open synchronous mode
vrep_sim.simxSynchronous(client_ID, True) 
# Start simulation
vrep_sim.simxStartSimulation(client_ID, vrep_sim.simx_opmode_oneshot)

# ------------------------------- Initialize simulation model ------------------------------- 
UR5_sim_model = UR5SimModel()
UR5_sim_model.initializeSimModel(client_ID)


return_code, initial_dummy_handle = vrep_sim.simxGetObjectHandle(client_ID, 'initial', vrep_sim.simx_opmode_blocking)
if (return_code == vrep_sim.simx_return_ok):
    print('get initial dummy handle ok.')

return_code, goal_dummy_handle = vrep_sim.simxGetObjectHandle(client_ID, 'goal', vrep_sim.simx_opmode_blocking)
if (return_code == vrep_sim.simx_return_ok):
    print('get goal dummy handle ok.')

return_code, UR5_target_dummy_handle = vrep_sim.simxGetObjectHandle(client_ID, 'UR5_target', vrep_sim.simx_opmode_blocking)
if (return_code == vrep_sim.simx_return_ok):
    print('get UR5 target dummy handle ok.')

return_code, via_dummy_handle = vrep_sim.simxGetObjectHandle(client_ID, 'via', vrep_sim.simx_opmode_blocking)
if (return_code == vrep_sim.simx_return_ok):
    print('get via dummy handle ok.')

time.sleep(0.1)


# 创建以当前时间为名的文件夹
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = f"output_{current_time}"
os.makedirs(output_dir, exist_ok=True)
print(f"创建输出文件夹: {output_dir}")
from matplotlib import font_manager
font_path = './msyh.ttf'  # 替换为你的字体文件路径
font_prop = font_manager.FontProperties(fname=font_path)


#%% DMP learning
# get demonstrated trajectory from file
df = pd.read_csv('./demo_trajectory/demo_trajectory_for_discrete_dmp.csv', header=None)
reference_trajectory = np.array(df)
data_dim = reference_trajectory.shape[0]
data_len = reference_trajectory.shape[1]
# data_dim: 3
# data_len: 697
# (3, 697)

# n_bfs激活函数的数量是不是最好和step步数保持在一个数量级
dmp = dmp_discrete(n_dmps=data_dim, n_bfs=1000, dt=1.0/data_len)
dmp.learning(reference_trajectory)

reproduced_trajectory, _, _ = dmp.reproduce()

# 保存第一个3D图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=20, azim=45)
ax.grid(True)
ax.plot(reference_trajectory[0,:], reference_trajectory[1,:], reference_trajectory[2,:], 'g', label='reference', linewidth=2)
ax.plot(reproduced_trajectory[:,0], reproduced_trajectory[:,1], reproduced_trajectory[:,2], 'r--', label='reproduce', linewidth=2)
ax.legend()
ax.set_title('DMP 3D轨迹对比', fontproperties=font_prop)
ax.set_xlabel('X位置', fontproperties=font_prop)
ax.set_ylabel('Y位置', fontproperties=font_prop)
ax.set_zlabel('Z位置', fontproperties=font_prop)
plt.savefig(f'{output_dir}/dmp_3d_trajectory_comparison.png', dpi=300, bbox_inches='tight')
plt.close()

# 保存多子图
fig = plt.figure(figsize=(10, 8))
plt.subplot(311)
plt.plot(reference_trajectory[0,:], 'g', label='reference')
plt.plot(reproduced_trajectory[:,0], 'r--', label='reproduce')
plt.legend()
plt.title('X轴轨迹对比', fontproperties=font_prop)
plt.ylabel('X位置', fontproperties=font_prop)

plt.subplot(312)
plt.plot(reference_trajectory[1,:], 'g', label='reference')
plt.plot(reproduced_trajectory[:,1], 'r--', label='reproduce')
plt.legend()
plt.title('Y轴轨迹对比', fontproperties=font_prop)
plt.ylabel('Y位置', fontproperties=font_prop)

plt.subplot(313)
plt.plot(reference_trajectory[2,:], 'g', label='reference')
plt.plot(reproduced_trajectory[:,2], 'r--', label='reproduce')
plt.legend()
plt.title('Z轴轨迹对比', fontproperties=font_prop)
plt.ylabel('Z位置', fontproperties=font_prop)
plt.xlabel('时间步', fontproperties=font_prop)

plt.tight_layout()
plt.savefig(f'{output_dir}/dmp_trajectory_components.png', dpi=300, bbox_inches='tight')
plt.close()

#%% the main loop
print("Main loop is begining ...")
max_loop = 10

reproduced_trajectory_record_x = np.zeros((data_len, max_loop))
reproduced_trajectory_record_y = np.zeros((data_len, max_loop))
reproduced_trajectory_record_z = np.zeros((data_len, max_loop))

for loop in range(max_loop):
    print('loop: %d' % loop)
    if loop == 0:
        # DMP reproduce the reference trajectory
        reproduced_trajectory, _, _ = dmp.reproduce()
    elif loop <= 5:
        # randomly add offset to the trajectory initial and goal positions
        goal_pos = [reference_trajectory[0,-1] + np.random.uniform(-0.1,0.2), reference_trajectory[1,-1]  + np.random.uniform(-0.2,0.05), reference_trajectory[2,-1] + np.random.uniform(-0.5, 0.1)]
        vrep_sim.simxSetObjectPosition(client_ID, goal_dummy_handle, -1, goal_pos, vrep_sim.simx_opmode_oneshot)

        # DMP reproduce with new goal positions
        reproduced_trajectory, _, _ = dmp.reproduce(goal=goal_pos)
    else:
        # randomly add offset to the trajectory initial and goal positions
        initial_pos = [reference_trajectory[0,0] + np.random.uniform(-0.05,0.05), reference_trajectory[1,0]  + np.random.uniform(-0.05,0.05), reference_trajectory[2,0] + np.random.uniform(-0.05, 0.05)]
        vrep_sim.simxSetObjectPosition(client_ID, initial_dummy_handle, -1, initial_pos, vrep_sim.simx_opmode_oneshot)

        goal_pos = [reference_trajectory[0,-1] + np.random.uniform(-0.1,0.2), reference_trajectory[1,-1]  + np.random.uniform(-0.2,0.05), reference_trajectory[2,-1] + np.random.uniform(-0.5, 0.1)]
        vrep_sim.simxSetObjectPosition(client_ID, goal_dummy_handle, -1, goal_pos, vrep_sim.simx_opmode_oneshot)

        # DMP reproduce with new initial and goal positions
        reproduced_trajectory, _, _ = dmp.reproduce(initial=initial_pos, goal=goal_pos)

    current_data_len = reproduced_trajectory.shape[0]
    # 确保数组大小匹配
    min_len = min(data_len, current_data_len)
    reproduced_trajectory_record_x[:min_len,loop] = reproduced_trajectory[:min_len,0]
    reproduced_trajectory_record_y[:min_len,loop] = reproduced_trajectory[:min_len,1]
    reproduced_trajectory_record_z[:min_len,loop] = reproduced_trajectory[:min_len,2]
    print(f'Loop {loop}: 记录轨迹数据，长度: {min_len}')

    # go to the goal position
    for i in range(current_data_len):
        UR5_target_pos = reproduced_trajectory[i,:]
        vrep_sim.simxSetObjectPosition(client_ID, via_dummy_handle, -1, UR5_target_pos, vrep_sim.simx_opmode_oneshot)
        vrep_sim.simxSynchronousTrigger(client_ID)  # trigger one simulation step

    # go back to the initial position
    for i in range(current_data_len-1, 0, -1):
        UR5_target_pos = reproduced_trajectory[i,:]
        vrep_sim.simxSetObjectPosition(client_ID, via_dummy_handle, -1, UR5_target_pos, vrep_sim.simx_opmode_oneshot)
        vrep_sim.simxSynchronousTrigger(client_ID)  # trigger one simulation step

vrep_sim.simxStopSimulation(client_ID, vrep_sim.simx_opmode_blocking) # stop the simulation
vrep_sim.simxFinish(-1)  # Close the connection
print('Program terminated')

#%% Plot
# 保存最终的3D图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=20, azim=45)
ax.grid(True)
#目前只绘制参考轨迹。无法给出改变起始点和目标点后的参照轨迹
ax.plot(reference_trajectory[0,:], reference_trajectory[1,:], reference_trajectory[2,:], 'g', label='reference', linewidth=2)

# 绘制所有记录的轨迹
for i in range(max_loop):
    # 检查是否有非零数据
    if np.any(reproduced_trajectory_record_x[:,i] != 0):
        ax.plot(reproduced_trajectory_record_x[:,i], reproduced_trajectory_record_y[:,i], reproduced_trajectory_record_z[:,i], '--', alpha=0.7, label=f'reproduce_{i+1}')

ax.legend()
ax.set_xlabel('X位置', fontproperties=font_prop)
ax.set_ylabel('Y位置', fontproperties=font_prop)
ax.set_zlabel('Z位置', fontproperties=font_prop)
ax.set_title('DMP多轮轨迹对比', fontproperties=font_prop)
plt.savefig(f'{output_dir}/dmp_multiple_trajectories.png', dpi=300, bbox_inches='tight')
plt.close()

print(f"记录的轨迹数据形状: {reproduced_trajectory_record_x.shape}")
print(f"非零轨迹数量: {np.sum(np.any(reproduced_trajectory_record_x != 0, axis=0))}")

print(f"所有图片已保存到文件夹: {output_dir}")
