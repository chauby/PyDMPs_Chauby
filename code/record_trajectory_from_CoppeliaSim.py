# -*- coding: utf-8 -*-

#%% import package
import time
import numpy as np

import sys
sys.path.append('./UR5/VREP_RemoteAPIs')
import sim as vrep_sim

sys.path.append('./UR5')
from UR5SimModel import UR5SimModel

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pandas as pd

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

delta_t = 0.005 # simulation time step
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

time.sleep(0.1)

# get initial and goal position from CoppeliaSim 
_, initial_pos = vrep_sim.simxGetObjectPosition(client_ID, initial_dummy_handle, -1, vrep_sim.simx_opmode_blocking)

_, goal_pos = vrep_sim.simxGetObjectPosition(client_ID, goal_dummy_handle, -1, vrep_sim.simx_opmode_blocking)

pos_record_x = list()
pos_record_y = list()
pos_record_z = list()

record_enable = False

while True:
    _, UR5_target_pos = vrep_sim.simxGetObjectPosition(client_ID, UR5_target_dummy_handle, -1, vrep_sim.simx_opmode_oneshot)

    if (record_enable == False) and (np.sqrt((UR5_target_pos[0] - initial_pos[0])**2 + (UR5_target_pos[1] - initial_pos[1])**2 + (UR5_target_pos[2] - initial_pos[2])**2) < 0.005):
        record_enable = True
    if (np.sqrt((UR5_target_pos[0] - goal_pos[0])**2 + (UR5_target_pos[1] - goal_pos[1])**2 + (UR5_target_pos[2] - goal_pos[2])**2) < 0.005):
        record_enable = False
        break

    if record_enable == True:
        pos_record_x.append(UR5_target_pos[0])
        pos_record_y.append(UR5_target_pos[1])
        pos_record_z.append(UR5_target_pos[2])

    vrep_sim.simxSynchronousTrigger(client_ID)  # trigger one simulation step

vrep_sim.simxStopSimulation(client_ID, vrep_sim.simx_opmode_blocking) # stop the simulation
vrep_sim.simxFinish(-1)  # Close the connection
print('Program terminated')

print(len(pos_record_x))

fig = plt.figure()
ax=Axes3D(fig)
plt.plot(pos_record_x, pos_record_y, pos_record_z)
plt.show()

#%% save the recorded data to files
data = np.vstack((pos_record_x, pos_record_y, pos_record_z))
print(data)

df = pd.DataFrame(data)
df.to_csv('./demo_trajectory/demo_trajectory_for_discrete_dmp.csv', index=False, header=None)
