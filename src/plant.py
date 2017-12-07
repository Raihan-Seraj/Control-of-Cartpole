'''
Copyright (c) 2017, Juan Camilo Gamboa Higuera, Anqi Xu, Victor Barbaros, Alex Chatron-Michaud, David Meger

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import numpy as np
import sys
import serial
import struct
from enum import Enum

from matplotlib import pyplot as plt
from matplotlib.widgets import Cursor
from matplotlib.colors import cnames
from scipy.integrate import ode
from time import time, sleep
from threading import Thread, Lock
from multiprocessing import Process,Pipe,Event

color_generator = cnames.iteritems()

def gTrig_np(x,angi):
    if type(x) is list:
        x = np.array(x)
    if x.ndim == 1:
        x = x[None,:]
    D = x.shape[1]
    Da = 2*len(angi)
    n = x.shape[0]
    xang = np.zeros((n,Da))
    xi = x[:,angi]
    xang[:,::2] =  np.sin(xi)
    xang[:,1::2] =  np.cos(xi)

    non_angle_dims = list(set(range(D)).difference(angi))
    xnang = x[:,non_angle_dims]
    m = np.concatenate([xnang,xang],axis=1)

    return m

class Plant(object):
    def __init__(self, params=None, x0=None, S0=None, dt=0.01, noise=None, name='Plant', angle_dims = []):
        self.name = name
        self.params = params
        self.x0 = x0
        self.S0= S0
        self.x = np.array(x0,dtype=np.float64).flatten()
        self.u = None
        self.t = 0
        self.dt = dt
        self.noise = noise
        self.running = Event()
        self.done = False
        self.plant_thread = None
        self.angle_dims = angle_dims
    
    def apply_control(self,u):
        self.u = np.array(u,dtype=np.float64)
        if len(self.u.shape) < 2:
            self.u = self.u[:,None]

    def get_plant_state(self):
        if self.angle_dims is None:
            return self.x.flatten(),self.t
        else:
            return gTrig_np(self.x, self.angle_dims).flatten(), self.t
    
    def run(self):
        start_time = time()
        print('Starting plant loop',self.name)
        while self.running.is_set():
            exec_time = time()
            self.step(self.dt)
            exec_time = time() - exec_time
            sleep(max(self.dt-exec_time,0))

    def start(self):
        if self.plant_thread is not None and self.plant_thread.is_alive():
            while self.plant_thread.is_alive():
                sleep(1.0)
        
        self.plant_thread = Thread(target=self.run)
        self.plant_thread.daemon = True
        self.running.set()
        self.plant_thread.start()

    def stop(self):
        self.running.clear()
        if self.plant_thread is not None and self.plant_thread.is_alive():
            # wait until thread stops
            self.plant_thread.join(10)
            # create new thread object, since python threads can only be started once
            self.plant_thread = Thread(target=self.run)
            self.plant_thread.daemon = True

    def step(self):
        raise NotImplementedError("You need to implement the step method in your Plant subclass.")

    def reset_state(self):
        raise NotImplementedError("You need to implement the reset_state method in your Plant subclass.")

class ODEPlant(Plant):
    def __init__(self, params, x0, S0=None, dt=0.01, noise=None, name='ODEPlant', integrator='dopri5', atol=1e-12, rtol=1e-12, angle_dims = []):
        super(ODEPlant,self).__init__(params, x0, S0, dt, noise, name, angle_dims)
        self.solver = ode(self.dynamics).set_integrator(integrator,atol=atol,rtol=rtol)
        self.set_state(self.x0)

    def set_state(self, x):
        if (self.x is None or np.linalg.norm(x-self.x) > 1e-12):
            self.x = np.array(x,dtype=np.float64).flatten()
        self.solver = self.solver.set_initial_value(self.x)
        self.t = self.solver.t
    
    def reset_state(self):
        if self.S0 is None:
            self.set_state(self.x0)
        else:
            #self.set_state(np.random.multivariate_normal(self.x0,self.S0))
            L_noise = np.linalg.cholesky(self.S0)
            start = self.x0 + np.random.randn(self.S0.shape[1]).dot(L_noise)
            self.set_state( start );

    def step(self,dt=None):
        if dt is None:
            dt = self.dt
        t1 = self.solver.t + dt
        while self.solver.successful and self.solver.t < t1:
            self.solver.integrate(self.solver.t+ dt)
        self.x = np.array(self.solver.y)
        self.t = self.solver.t
        return self.x

    def dynamics(self):
        raise NotImplementedError("You need to implement the dynamics method in your ODEPlant subclass.")

class SerialPlant(Plant):
    cmds = ['RESET_STATE','GET_STATE','APPLY_CONTROL','CMD_OK','STATE']
    cmds = dict(zip(cmds,[str(i) for i in xrange(len(cmds))]))

    def __init__(self, params=None, x0=None, S0=None, dt=0.1, noise=None, name='SerialPlant', baud_rate=115200, port='/dev/ttyACM0', state_indices=None, maxU=None, angle_dims = []):
        super(SerialPlant,self).__init__(params, x0, S0, dt, noise, name, angle_dims)
        self.port = port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(self.port,self.baud_rate)
        self.state_indices = state_indices if state_indices is not None else range(len(x0))
        self.U_scaling = 1.0/np.array(maxU);
        self.t=-1
    
    def apply_control(self,u):
        if not self.serial.isOpen():
            self.serial.open()
        self.u = np.array(u,dtype=np.float64)
        if len(self.u.shape) < 2:
            self.u = self.u[:,None]
        if self.U_scaling is not None:
            self.u *= self.U_scaling;
        if self.t < 0:
            self.x,self.t= self.state_from_serial()

        u_array = self.u.flatten().tolist()
        u_array.append(self.t+self.dt)
        u_string = ','.join([ str(ui) for ui in u_array ] ) #TODO pack as binary
        self.serial.flushInput()
        self.serial.flushOutput()
        cmd = self.cmds['APPLY_CONTROL']+','+u_string+";"
        self.serial.write(cmd)

    def step(self,dt=None):
        if not self.serial.isOpen():
            self.serial.open()
        if dt is None:
            dt = self.dt
        t1 = self.t + dt
        while self.t < t1:
            self.x,self.t= self.state_from_serial()
        return self.x

    def state_from_serial(self):
        self.serial.flushInput()
        self.serial.write(self.cmds['GET_STATE']+";")
        c = self.serial.read()
        buf = c
        while buf != self.cmds['STATE']+',': # TODO timeout this loop
            c = self.serial.read()
            buf = buf[-1]+c
        buf = []
        res = []
        escaped = False
        while True: # TODO timeout this loop
            c = self.serial.read()
            if not escaped:
                if c == '/':
                    escaped = True
                    continue
                elif c == ',':
                    res.append(''.join(buf))
                    buf = []
                    continue
                elif c == ';':
                    res.append(''.join(buf))
                    buf = []
                    break
            buf.append(c)
            escaped = False

        res = np.array([struct.unpack('<d',ri) for ri in res]).flatten()
        return res[self.state_indices],res[-1]

    def reset_state(self):
        raw_input()
        if not self.serial.isOpen():
            self.serial.open()
        self.serial.flushInput()
        self.serial.flushOutput()
        self.serial.write(self.cmds['RESET_STATE']+";")
        sleep(self.dt)
        self.x,self.t= self.state_from_serial()
        self.t=-1

    def stop(self):
        super(SerialPlant,self).stop()
        self.serial.close()

class PlantDraw(object):
    def __init__(self, plant, refresh_period=(1.0/24), name='PlantDraw'):
        super(PlantDraw,self).__init__()
        self.name = name
        self.plant = plant
        self.drawing_thread=None
        self.polling_thread=None

        self.dt = refresh_period
        self.scale =  150 # pixels per meter

        self.center_x = 0
        self.center_y = 0
        self.running = Event()

        self.polling_pipe,self.drawing_pipe = Pipe()

    def init_ui(self):
        self.fig = plt.figure(self.name)#,figsize=(16,10))
        plt.xlim([-1.5,1.5])
        plt.ylim([-1.5,1.5])
        self.ax = plt.gca()
        self.ax.set_aspect('equal','datalim')
        self.ax.grid(True)
        self.bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self.init_artists()
        self.fig.canvas.draw()
        self.cursor = Cursor(self.ax, useblit=True, color='red', linewidth=2 )
        plt.ion()
        plt.show()

    def drawing_loop(self,drawing_pipe):
        # start the matplotlib plotting
        self.init_ui()

        while self.running.is_set():
            exec_time = time()
            
            # get any data from the polling loop
            updts = None
            while drawing_pipe.poll():
                data_from_plant= drawing_pipe.recv()
                if data_from_plant is None:
                    self.running.clear()
                    break
                
                # get the visuzlization updates from the latest state
                state,t = data_from_plant
                updts = self.update(state,t)

            if updts is not None:
                # update the drawing from the plant state
                self.fig.canvas.restore_region(self.bg)

                for artist in updts:
                    self.ax.draw_artist(artist)
                self.fig.canvas.blit(self.ax.bbox)

            # sleep to guarantee the desired frame rate
            exec_time = time() - exec_time
            sleep(max(self.dt-exec_time,0))

        # close the matplotlib windows, clean up
        plt.ioff()
        plt.close(self.fig)

    def polling_loop(self,polling_pipe):
        current_t = -1
        while self.running.is_set():
            exec_time = time()
            state, t = self.plant.get_plant_state()
            if t != current_t:
                polling_pipe.send((state,t))

            # sleep to guarantee the desired frame rate
            exec_time = time() - exec_time
            sleep(max(self.dt-exec_time,0))

    def start(self):
        self.drawing_thread = Process(target=self.drawing_loop,args=(self.drawing_pipe,))
        self.drawing_thread.daemon = True
        self.polling_thread = Thread(target=self.polling_loop,args=(self.polling_pipe,))
        self.polling_thread.daemon = True
        #self.drawing_thread = Process(target=self.run)
        self.running.set()
        self.polling_thread.start()
        self.drawing_thread.start()
    
    def stop(self):
        self.running.clear()

        if self.drawing_thread is not None and self.drawing_thread.is_alive():
            # wait until thread stops
            self.drawing_thread.join(10)

        if self.polling_thread is not None and self.polling_thread.is_alive():
            # wait until thread stops
            self.polling_thread.join(10)

    def update(self):
        raise NotImplementedError("You need to implement the self.update() method in your PlantDraw class.")
    
    def init_artists(self):
        raise NotImplementedError("You need to implement the self.init_artists() method in your PlantDraw class.")

# an example that plots lines
class LivePlot(PlantDraw):
    def __init__(self, plant, refresh_period=1.0, name='Serial Data',H=5.0, angi=[]):
        super(LivePlot, self).__init__(plant, refresh_period,name)
        self.H = H
        self.angi = angi
        # get first measurement
        state, t = plant.get_plant_state()
        self.data = np.array([state])
        self.t_labels = np.array([t])
        
        # keep track of latest time stamp and state
        self.current_t = t
        self.previous_update_time = time()
        self.update_period = refresh_period

    def init_artists(self):
        self.lines =[ plt.Line2D(self.t_labels,self.data[:,i], c=color_generator.next()[0]) for i in xrange(self.data.shape[1]) ]
        self.ax.set_aspect('auto','datalim')
        for line in self.lines:
            self.ax.add_line(line)
        self.previous_update_time = time()

    def update(self, state, t):
        if t!=self.current_t:
            if len(self.data)<=1:
                self.data = np.array([state]*2)
                self.t_labels = np.array([t]*2)

            if len(self.angi)>0:
                state[self.angi] = (state[self.angi] + np.pi) % (2 * np.pi ) - np.pi

            self.current_t = t
            # only keep enough data points to fill the window to avoid using up too much memory
            curr_time = time()
            self.update_period = 0.95*self.update_period + 0.05*(curr_time-self.previous_update_time)
            self.previous_update_time = curr_time
            history_size = int(1.5*self.H/self.update_period)
            self.data = np.vstack((self.data,state))[-history_size:,:]
            self.t_labels = np.append(self.t_labels,t)[-history_size:]

            # update the lines
            for i in xrange(len(self.lines)):
                self.lines[i].set_data(self.t_labels,self.data[:,i])

            # update the plot limits
            plt.xlim([self.t_labels.min(),self.t_labels.max()])
            plt.xlim([t-self.H,t])
            mm = self.data.mean()
            ll = 1.05*np.abs(self.data[:,:]).max()
            plt.ylim([mm-ll,mm+ll])
            self.ax.autoscale_view(tight=True,scalex=True,scaley=True)

        return self.lines
