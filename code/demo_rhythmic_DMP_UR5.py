# -*- coding: utf-8 -*-
import time
import numpy as np

import sys
sys.path.append('./UR5/VREP_RemoteAPIs')
import sim as vrep_sim

sys.path.append('./UR5')
from UR5SimModel import UR5SimModel

sys.path.append('./DMP')
from dmp_rhythmic import dmp_rhythmic

import matplotlib.pyplot as plt

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
time.sleep(0.1)

t = np.linspace(0, 2*np.pi, 800)
data_len = len(t)
reference_q = np.zeros((3, data_len))

reference_q[0,:] = np.sin(t)
reference_q[1,:] = np.sin(t)
reference_q[2,:] = np.sin(t)

dmp = dmp_rhythmic(n_dmps=reference_q.shape[0], n_bfs=200, dt=2*np.pi/data_len)
dmp.learning(reference_q)

max_loop = 3
current_q1_record = list()
current_q2_record = list()
current_q3_record = list()

desired_q1_record = list()
desired_q2_record = list()
desired_q3_record = list()

# set different parameters for each loop
tau = [1.0, 0.8, 1.2]

goal_1 = [0.0, 0.2, 0]
goal_2 = [0.0, 0.2, 0]
goal_3 = [0.0, 0.2, 0]

r_1 = [1.0, 0.5, 1.2]
r_2 = [1.0, 1.2, 0.5]
r_3 = [1.0, 0.8, 0.8]

print("Main loop is begining ...")
for loop in range(max_loop):
    goal = [goal_1[loop], goal_2[loop], goal_3[loop]]
    r = [r_1[loop], r_2[loop], r_3[loop]]
    desired_q, _, _ = dmp.reproduce(tau=tau[loop], goal=goal, r=r)

    data_len = desired_q.shape[0]
    print(data_len)
    for i in range(data_len):
        UR5_sim_model.setJointAngle('UR5_joint3', desired_q[i, 0])
        UR5_sim_model.setJointAngle('UR5_joint4', desired_q[i, 1])
        UR5_sim_model.setJointAngle('UR5_joint5', desired_q[i, 2])

        desired_q1_record.append(desired_q[i, 0])
        desired_q2_record.append(desired_q[i, 1])
        desired_q3_record.append(desired_q[i, 2])

        q = UR5_sim_model.getAllJointAngles()
        current_q1_record.append(q[2])
        current_q2_record.append(q[3])
        current_q3_record.append(q[4])

        vrep_sim.simxSynchronousTrigger(client_ID)  # trigger one simulation step

vrep_sim.simxStopSimulation(client_ID, vrep_sim.simx_opmode_blocking) # stop the simulation
vrep_sim.simxFinish(-1)  # Close the connection
print('Program terminated')

plt.figure(figsize=(10, 5))
plt.plot(desired_q1_record, 'g', label='desired q 1')
plt.plot(current_q1_record, 'r--', label='current q 1')
plt.plot(desired_q2_record, 'b', label='desired q 2')
plt.plot(current_q2_record, 'm--', label='current q 2')
plt.plot(desired_q3_record, 'k', label='desired q 3')
plt.plot(current_q3_record, 'y--', label='current q 3')
plt.legend()
plt.grid()
plt.xlabel('time')
plt.ylabel('joint angles')
plt.show()
