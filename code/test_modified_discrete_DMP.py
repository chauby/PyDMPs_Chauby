# -*- coding: utf-8 -*-
'''
This code is implemented by Chauby, it is free for everyone.
Email: chaubyZou@163.com
'''

#%% import package
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('./DMP')

# from dmp_discrete_original import dmp_discrete # original version
from dmp_discrete import dmp_discrete # modified version

# %%
data_len = 200

demo_traj = np.zeros((2, data_len))
demo_traj[0,:] = np.linspace(0, 1.5*np.pi, data_len)
demo_traj[1,:] = np.sin(demo_traj[0,:])

# %% ----------------- For same initial and goal positions
t = np.linspace(0, 1.0, data_len)

y_demo = np.zeros((2, data_len))
y_demo[0,:] = demo_traj[0,:]
y_demo[1,:] = demo_traj[1,:]

# DMP learning
dmp = dmp_discrete(n_dmps=y_demo.shape[0], n_bfs=200, dt=1.0/data_len)
dmp.learning(y_demo, plot=False)

# reproduce learned trajectory
y_reproduce, dy_reproduce, ddy_reproduce = dmp.reproduce()

# set new initial and goal positions
y_reproduce_2, dy_reproduce_2, ddy_reproduce_2 = dmp.reproduce(tau=1.0, initial=[y_demo[0,0]+1.0, y_demo[1,0]], goal=[y_demo[0, -1]-1.0, y_demo[1, -1]-1.0])

plt.figure(figsize=(10, 5))

plt.subplot(2,1,1)
plt.plot(y_demo[0,:], 'g', label='demo x')
plt.plot(y_reproduce[:,0], 'r', label='reproduce x')
plt.plot(y_reproduce_2[:,0], 'r-.', label='reproduce 2 x')
plt.plot(y_demo[1,:], 'b', label='demo y')
plt.plot(y_reproduce[:,1], 'm', label='reproduce y')
plt.plot(y_reproduce_2[:,1], 'm-.', label='reproduce 2 y')
plt.legend(loc="upper right")
# plt.ylim(-1.5, 3)
plt.grid()
plt.xlabel('time')
plt.ylabel('y')

plt.subplot(2,1,2)
plt.plot(y_demo[0,:], y_demo[1,:], 'g', label='demo')
plt.plot(y_reproduce[:,0], y_reproduce[:,1], 'r', label='reproduce')
plt.plot(y_reproduce_2[:,0], y_reproduce_2[:,1], 'r-.', label='reproduce 2')
plt.legend(loc="upper right")
# plt.ylim(-1.5, 3)
plt.grid()
plt.xlabel('x')
plt.ylabel('y')
plt.show()

# %%
