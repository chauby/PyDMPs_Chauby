# -*- coding: utf-8 -*-
'''
This code is implemented by Chauby, it is free for everyone.
Email: chaubyZou@163.com
'''

#%% import package
import numpy as np

#%% define canonical system
class CanonicalSystem():
    def __init__(self, alpha_x=1.0, dt=0.01, type='discrete'):
        self.x = 1.0
        self.alpha_x = alpha_x
        self.dt = dt
        self.dmp_type = type

        if type == 'discrete':
            self.run_time = 1.0
        elif type == 'rhythmic':
            self.run_time = 2*np.pi
        else:
            print('Initialize Canonical system failed, can not recognize DMP type: ' + type)
        
        self.timesteps = round(self.run_time/self.dt)
        self.reset_state()

    def run(self, **kwargs): # run to goal state
        if 'tau' in kwargs:
            timesteps = int(self.timesteps / kwargs['tau'])
        else:
            timesteps = self.timesteps

        self.reset_state()
        self.x_track = np.zeros(timesteps)

        if self.dmp_type == 'discrete':
            for t in range(timesteps):
                self.x_track[t] = self.x
                self.step_discrete(**kwargs)
        elif self.dmp_type == 'rhythmic':
            for t in range(timesteps):
                self.x_track[t] = self.x
                self.step_rhythmic(**kwargs)

        return self.x_track
    
    def reset_state(self): # reset state
        self.x = 1.0

    def step_discrete(self, tau=1.0):
        dx = -self.alpha_x*self.x*self.dt
        self.x += tau*dx
        return self.x

    def step_rhythmic(self, tau=1.0):
        self.x += tau*self.dt
        return self.x


#%% test code
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    cs_1 = CanonicalSystem(alpha_x=0.5, dt=0.01)
    x_track_1 = cs_1.run(tau=0.5)
    x_track_2 = cs_1.run(tau=1.0)
    x_track_3 = cs_1.run(tau=2.0)

    cs_2 = CanonicalSystem(alpha_x=1.5, dt=0.01)
    x_track_4 = cs_2.run(tau=0.5)
    x_track_5 = cs_2.run(tau=1.0)
    x_track_6 = cs_2.run(tau=2.0)

    cs_3 = CanonicalSystem(alpha_x=5.0, dt=0.01)
    x_track_7 = cs_3.run(tau=0.5)
    x_track_8 = cs_3.run(tau=1.0)
    x_track_9 = cs_3.run(tau=2.0)

    plt.figure(figsize=(10, 5))
    plt.plot(x_track_1, 'g--', label='alpha_x=0.5, tau=0.5')
    plt.plot(x_track_2, 'r--', label='alpha_x=0.5, tau=1.0')
    plt.plot(x_track_3, 'b--', label='alpha_x=0.5, tau=2.0')

    plt.plot(x_track_4, 'g-.', label='alpha_x=1.5, tau=0.5')
    plt.plot(x_track_5, 'r-.', label='alpha_x=1.5, tau=1.0')
    plt.plot(x_track_6, 'b-.', label='alpha_x=1.5, tau=2.0')

    plt.plot(x_track_7, 'g', label='alpha_x=5.0, tau=0.5')
    plt.plot(x_track_8, 'r', label='alpha_x=5.0, tau=1.0')
    plt.plot(x_track_9, 'b', label='alpha_x=5.0, tau=2.0')

    plt.legend()
    plt.grid()
    plt.xlabel('time')
    plt.ylabel('x')
    plt.xlim(-10, 250)
    plt.ylim(-0.1, 1.1)
    plt.show()
