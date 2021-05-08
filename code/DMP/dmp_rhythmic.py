# -*- coding: utf-8 -*-
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

from cs import CanonicalSystem

#%% define rhythmic dmp
class dmp_rhythmic():
    def __init__(self, n_dmps=1, n_bfs=100, dt=0, alpha_y=None, beta_y=None, **kwargs):
        self.n_dmps = n_dmps # number of data dimensions, one dmp for one degree
        self.n_bfs = n_bfs # number of basis functions
        self.dt = dt

        self.goal = np.ones(n_dmps) # for multiple dimensions

        self.alpha_y = np.ones(n_dmps) * 25.0 if alpha_y is None else alpha_y
        self.beta_y = self.alpha_y / 4.0 if beta_y is None else beta_y
        self.tau = 1.0

        self.w = np.zeros((n_dmps, n_bfs)) # weights for forcing term
        self.psi_centers = np.zeros(self.n_bfs) # centers over canonical system for Gaussian basis functions
        self.psi_h = np.zeros(self.n_bfs) # variance over canonical system for Gaussian basis functions

        # canonical system
        self.cs = CanonicalSystem(dt=self.dt, type="rhythmic")
        self.timesteps = round(self.cs.run_time / self.dt)

        # generate centers for Gaussian basis functions
        self.generate_centers()

        self.h = np.ones(self.n_bfs) * self.n_bfs

        # reset state
        self.reset_state()

    # Reset the system state
    def reset_state(self):
        self.y = np.zeros(self.n_dmps)
        self.dy = np.zeros(self.n_dmps)
        self.ddy = np.zeros(self.n_dmps)
        self.cs.reset_state()

    def generate_centers(self):
        self.psi_centers = np.linspace(0, 2*np.pi, self.n_bfs)
        return self.psi_centers

    def generate_psi(self, x):
        if isinstance(x, np.ndarray):
            x = x[:, None]
        self.psi = np.exp(self.h * (np.cos(x - self.psi_centers) - 1))

        return self.psi
    
    def generate_weights(self, f_target):
        x_track = self.cs.run()
        psi_track = self.generate_psi(x_track)

        for d in range(self.n_dmps):
            for b in range(self.n_bfs):
                # note that here r is the default value, r == 1
                numer = 1*np.sum(psi_track[:,b] * f_target[:,d])
                denom = np.sum(psi_track[:,b] + 1e-10)
                self.w[d, b] = numer / denom
        return self.w

    def learning(self, y_demo, plot=False):
        if y_demo.ndim == 1: # data is with only one dimension
            y_demo = y_demo.reshape(1, len(y_demo))

        # For rhythmic DMPs, the goal is the average of the desired trajectory
        goal = np.zeros(self.n_dmps)
        for n in range(self.n_dmps):
            num_idx = ~np.isnan(y_demo[n])  # ignore nan's when calculating goal
            goal[n] = 0.5 * (y_demo[n, num_idx].min() + y_demo[n, num_idx].max())
        self.goal = goal

        # interpolate the demonstrated trajectory to be the same length with timesteps
        x = np.linspace(0, self.cs.run_time, y_demo.shape[1])
        y = np.zeros((self.n_dmps, self.timesteps))
        for d in range(self.n_dmps):
            y_tmp = interp1d(x, y_demo[d])
            for t in range(self.timesteps):
                y[d, t] = y_tmp(t*self.dt)
        
        # calculate velocity and acceleration of y_demo
        dy_demo = np.gradient(y, axis=1) / self.dt
        ddy_demo = np.gradient(dy_demo, axis=1) / self.dt

        f_target = np.zeros((y_demo.shape[1], self.n_dmps))
        for d in range(self.n_dmps):
            f_target[:,d] = ddy_demo[d] - (self.alpha_y[d]*(self.beta_y[d]*(self.goal[d] - y_demo[d]) - dy_demo[d]))
        
        self.generate_weights(f_target)

        if plot is True:
            # plot the basis function activations
            plt.figure()
            plt.subplot(211)
            psi_track = self.generate_psi(self.cs.run())
            plt.plot(psi_track)
            plt.title('basis functions')

            # plot the desired forcing function vs approx
            plt.subplot(212)
            plt.plot(f_target[:,0])
            plt.plot(np.sum(psi_track * self.w[0], axis=1) * self.dt)
            plt.legend(['f_target', 'w*psi'])
            plt.title('DMP forcing function')
            plt.tight_layout()
            plt.show()

        # reset state
        self.reset_state()


    def reproduce(self, tau=None, goal=None, r=None):
        # set temporal scaling
        if tau == None:
            timesteps = self.timesteps
        else:
            timesteps = round(self.timesteps/tau)

        # set goal state
        if goal != None:
            self.goal = goal
        
        # set r
        if r == None:
            r = np.ones(self.n_dmps)
        
        # reset state
        self.reset_state()

        y_reproduce = np.zeros((timesteps, self.n_dmps))
        dy_reproduce = np.zeros((timesteps, self.n_dmps))
        ddy_reproduce = np.zeros((timesteps, self.n_dmps))

        for t in range(timesteps):
            y_reproduce[t], dy_reproduce[t], ddy_reproduce[t] = self.step(tau=tau, r=r)
        
        return y_reproduce, dy_reproduce, ddy_reproduce

    def step(self, tau=None, r=None):
        # run canonical system
        if tau == None:
            tau = self.tau
        x = self.cs.step_rhythmic(tau)

        # generate basis function activation
        psi = self.generate_psi(x)

        for d in range(self.n_dmps):
            # generate forcing term
            f = r[d]*np.dot(psi, self.w[d]) / np.sum(psi)

            # generate reproduced trajectory
            self.ddy[d] = (tau**2)*(self.alpha_y[d]*(self.beta_y[d]*(self.goal[d] - self.y[d]) - self.dy[d]/tau) + f)
            self.dy[d] += tau*self.ddy[d]*self.dt
            self.y[d] += self.dy[d]*self.dt
        
        return self.y, self.dy, self.ddy

#%% test code
if __name__ == "__main__":
    #%% --------------------- For trajectory with one dimension
    data_len = 1000
    t = np.linspace(0, 3.0, data_len)
    y_demo = np.sin(2*np.pi*t) + 0.25*np.cos(4*np.pi*t + 0.77) + 0.1*np.sin(6*np.pi*t + 3.0)

    # DMP learning
    dmp = dmp_rhythmic(n_dmps=1, n_bfs=200, dt=2*np.pi/data_len)
    dmp.learning(y_demo)

    # DMP reproduce
    y_reproduce, dy_reproduce, ddy_reproduce = dmp.reproduce()

    # DMP reproduce with new goals and different r
    y_reproduce_2, dy_reproduce_2, ddy_reproduce_2 = dmp.reproduce(tau=0.8, goal=[-1.0], r=[0.8])
    y_reproduce_3, dy_reproduce_3, ddy_reproduce_3 = dmp.reproduce(tau=1.2, goal=[1.0], r=[1.5])

    plt.figure(figsize=(10, 6))
    plt.plot(y_demo, 'g', label="demo")
    plt.plot(y_reproduce, 'r--', label="reproduce")
    plt.plot(y_reproduce_2, 'm--', label="reproduce tau=0.8, goal=-1.0, r=0.8")
    plt.plot(y_reproduce_3, 'b--', label="reproduce tau=1.2, goal=1.0, r=1.5")
    plt.legend(loc='upper right')
    plt.grid()


    #%% --------------------- For trajectory with two dimensions
    data_len = 1000
    t = np.linspace(0, 5.0, data_len)
    y_demo = np.zeros((2, data_len))
    y_demo[0,:] = np.sin(2*np.pi*t) + 0.25*np.cos(4*np.pi*t + 0.77) + 0.1*np.sin(6*np.pi*t + 3.0)
    y_demo[1,:] = np.sin(2*np.pi*t) + 0.5*np.cos(np.pi*t)

    # DMP learning
    dmp = dmp_rhythmic(n_dmps=2, n_bfs=500, dt=2*np.pi/data_len)
    dmp.learning(y_demo)

    # DMP reproduce
    y_reproduce, dy_reproduce, ddy_reproduce = dmp.reproduce()

    # DMP reproduce with new goals and different r
    y_reproduce_2, dy_reproduce_2, ddy_reproduce_2 = dmp.reproduce(tau=1.0, goal=[-1.0, -1.0], r=[0.8, 0.5])
    y_reproduce_3, dy_reproduce_3, ddy_reproduce_3 = dmp.reproduce(tau=1.5, goal=[1.0, 1.0], r=[1.5, 1.2])

    plt.figure(figsize=(10, 10))
    plt.subplot(2,1,1)
    plt.plot(y_demo[0,:], 'g', label="demo")
    plt.plot(y_reproduce[:,0], 'r--', label="reproduce")
    plt.plot(y_reproduce_2[:,0], 'm--', label="reproduce tau=1.0, goal=-1.0, r=0.8")
    plt.plot(y_reproduce_3[:,0], 'b--', label="reproduce tau=1.0, goal=1.0, r=1.5")
    plt.legend(loc='upper right')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(y_demo[1,:], 'g', label="demo")
    plt.plot(y_reproduce[:,1], 'r--', label="reproduce")
    plt.plot(y_reproduce_2[:,1], 'm--', label="reproduce tau=1.5, goal=-1.0, r=0.5")
    plt.plot(y_reproduce_3[:,1], 'b--', label="reproduce tau=1.5, goal=1.0, r=1.2")
    plt.legend(loc='upper right')
    plt.grid()

    plt.show()
