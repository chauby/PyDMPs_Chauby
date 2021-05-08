# -*- coding: utf-8 -*-
import sys
sys.path.append('./VREP_RemoteAPIs')
import sim as vrep_sim

# UR5 simulation model in CoppeliaSim
class UR5SimModel():
    def __init__(self, name='UR5'):
        """
        :param: name: string
            name of objective
        """
        super(self.__class__, self).__init__()
        self.name = name
        self.client_ID = None

        # joint handle
        self.joint_handle = [-1, -1, -1, -1, -1, -1]

        # joint angles
        self.joint_angle = [0, 0, 0, 0, 0, 0]

    def initializeSimModel(self, client_ID):
        try:
            print ('Connected to remote API server')
            client_ID != -1
        except:
            print ('Failed connecting to remote API server')

        self.client_ID = client_ID

        for i in range(6):
            return_code, self.joint_handle[i] = vrep_sim.simxGetObjectHandle(client_ID, 'UR5_joint' + str(i+1), vrep_sim.simx_opmode_blocking)
            if (return_code == vrep_sim.simx_return_ok):
                print('get object joint handle ' + str(i+1) + ' ok.')

        # Get information from VREP for the first time
        for i in range(6):
            return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_handle[i], vrep_sim.simx_opmode_streaming)

        # Set the initialized position for each joint
        for i in range(6):
            vrep_sim.simxSetJointTargetPosition(self.client_ID, self.joint_handle[i], 0, vrep_sim.simx_opmode_streaming)

    
    def getJointAngle(self, joint_name):
        """
        :param: joint_name: str
            the joint name: can be UR5_joint1 ~ UR5_joint6
        """
        for i in range(6):
            if joint_name == 'UR5_joint' + str(i+1):
                return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_handle[i], vrep_sim.simx_opmode_buffer)
                return q
        
        # can not find the joint
        print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')
        return 0

    def getAllJointAngles(self):
        q = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            return_code, q[i] = vrep_sim.simxGetJointPosition(self.client_ID, self.joint_handle[i], vrep_sim.simx_opmode_buffer)

        return q

    def setJointAngle(self, joint_name, q):
        """
        :param: joint_name: str
            the joint name: can be UR5_joint1 ~ UR5_joint6
        :param: q: float array of size 6 x 1
            the desired joint angle for all joints
        """

        for i in range(6):
            if joint_name == 'UR5_joint' + str(i+1):
                vrep_sim.simxSetJointTargetPosition(self.client_ID, self.joint_handle[i], q, vrep_sim.simx_opmode_streaming)
                return
        
        # can not find the joint
        print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')
        return 0

#%% test code
if __name__ == "__main__":
    import time
    import numpy as np

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
    desired_q = [0, 0, 0, 0, 0, 0]

    print("Main loop is begining ...")
    t_max = 20
    for t in np.linspace(0, t_max, t_max*100):
        q = UR5_sim_model.getAllJointAngles()
        # print(q)

        desired_q = [0, 0, np.sin(t), 1.5*np.sin(t), 0.8*np.sin(t), 0]
        for i in range(6):
            UR5_sim_model.setJointAngle('UR5_joint'+ str(i+1), desired_q[i])

        vrep_sim.simxSynchronousTrigger(client_ID)  # trigger one simulation step

    vrep_sim.simxStopSimulation(client_ID, vrep_sim.simx_opmode_blocking) # stop the simulation
    vrep_sim.simxFinish(-1)  # Close the connection
    print('Program terminated')
