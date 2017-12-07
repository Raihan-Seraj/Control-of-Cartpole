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
import control
import math
from plant import gTrig_np
from cartpole import default_params
from cartpole import CartpoleDraw
import matplotlib.pyplot as plt
#np.random.seed(31337)
np.set_printoptions(linewidth=500)

# FOR YOU TODO: Fill in this function with a control 
# policy that computes a useful u from the input x
error_prev_pos=0
error_prev_angle=0
def policyfn( x,integral_pos,integral_angle,error_pos,error_angle,error_prev_pos,error_prev_angle):
    

    Kp_pos=1.105
    Kd_pos=-0.03*0
    Ki_pos=-1*1e-5*0

    Kp_angle=-11    # -9.7   #-10.244
    Kd_angle=-0.0017   #-0.0016 #-5
    Ki_angle=-0.0076   #-0.0076#-0.001*1

    error_pos=0-x[0]
    error_angle=0-x[3]
    print(error_angle)

    integral_pos+=error_pos*0.01
    integral_angle+=error_angle*0.01

    derivative_pos=(error_pos-error_prev_pos)/(0.01)
    derivative_angle=(error_angle-error_prev_angle)/(0.01)

    u_pos=(Kp_pos*error_pos)+(Kd_pos*derivative_pos)+(Ki_pos*integral_pos)
    u_angle=(Kp_angle*error_angle)+(Kd_angle*derivative_angle)+(Ki_angle*integral_angle)
    
    u=u_angle
    #print(u)
    return np.array([u]),integral_pos,integral_angle,error_pos,error_angle,error_prev_pos,error_prev_angle       

def LQR(plant,x):

	A = np.matrix(  [
					[0,1,0,0],
					[0,0,plant.params['g']*plant.params['m']/plant.params['M'],0],
					[0,0,0,1],
					[0,0,(plant.params['M']+plant.params['m'])*plant.params['g']/(plant.params['l']*plant.params['M']),0]
					])
	
	B = np.matrix(	[
					[0],
					[1/plant.params['M']],
					[0],
					[1/(plant.params['l']*plant.params['M'])]
					])

	# The Q and R matrices are emperically tuned. It is described further in the report
	Q = np.matrix(	[
					[1,0,0,0],
					[0,0.01,0,0],
					[0,0,15,0],
					[0,0,0,11]
					])

	R = np.matrix([5])

	# The K matrix is calculated using the lqr function from the controls library 
	K, S, E = control.lqr(A, B, Q, R)
	np.matrix(K)

	x = np.matrix(	[
					[-np.squeeze(np.asarray(x[0]))],
					[np.squeeze(np.asarray(x[1]))],
					[np.squeeze(np.asarray(np.arcsin(x[3])))],
					[-np.squeeze(np.asarray(x[2]))]
					])

	desired = np.matrix(	[
					[0],
					[0],
					[0],
					[0]
					])

	F = (K*(x-desired))
	return np.array(F)
    













def apply_controller(plant,params,H,policy=None):
    '''
    Starts the plant and applies the current policy to the plant for a duration specified by H (in seconds).

    @param plant is a class that controls our robot (simulation)
    @param params is a dictionary with some useful values 
    @param H Horizon for applying controller (in seconds)
    @param policy is a function pointer to the code that implements your 
            control solution. It will be called as u = policy( state )
    '''

    # start robot
    x_t, t = plant.get_plant_state()
    
    if plant.noise is not None:
        # randomize state
        Sx_t = np.zeros((x_t.shape[0],x_t.shape[0]))
        L_noise = np.linalg.cholesky(plant.noise)
        x_t = x_t + np.random.randn(x_t.shape[0]).dot(L_noise);
        
    
    sum_of_error = 0
    integral_pos=0
    integral_angle=0
    error_pos=0
    error_angle=0
    error_prev_pos=0
    error_prev_angle=0
    angle=[]
    position=[]
    #x_t_prev=gTrig_np(x_t[None,:], params['angle_dims']).flatten()
    H_steps = int(np.ceil(H/plant.dt))
    for i in range(H_steps):
        
        # convert input angle dimensions to complex representation
        x_t_ = gTrig_np(x_t[None,:], params['angle_dims']).flatten()
        #print(x_t_[4])
        #  get command from policy (this should be fast, or at least account for delays in processing):

        #u_t,integral_pos,integral_angle,error_pos,error_angle,error_prev_pos,error_prev_angle = policy(x_t_,integral_pos,integral_angle,error_pos,error_angle,error_prev_pos,error_prev_angle)
        u_t=LQR(plant,x_t_)
        error_prev_pos=error_pos
        error_prev_angle=error_angle
        angle.append(x_t_[3])
        position.append(x_t_[0])
       
	 #  send command to robot:
        plant.apply_control(u_t)
        plant.step()
        x_t, t = plant.get_plant_state()
        l = plant.params['l']
	err = np.array([0,l]) - np.array( [ x_t[0] + l*x_t_[3], -l*x_t_[4] ] )  
        dist = np.dot(err,err )
        sum_of_error = sum_of_error + dist

        if plant.noise is not None:
            # randomize state
            x_t = x_t + np.random.randn(x_t.shape[0]).dot(L_noise);
        
        if plant.done:
            break
    plt.plot(position)
    plt.ylim(-1,1)
    plt.title("Plot of position values with respect to steps ")
    plt.ylabel("Positions")
    plt.xlabel("steps")
    



    #plt.plot(position)
    plt.show()
    print "Error this episode %f"%(sum_of_error)
        
    # stop robot
    plant.stop()

def main():
    
    # learning iterations
    N = 1   
    H = 5
    
    learner_params = default_params()
    plant_params = learner_params['params']['plant'] 
    plant = learner_params['plant_class'](**plant_params)
   
    draw_cp = CartpoleDraw(plant)
    draw_cp.start()
    
    # loop to run controller repeatedly
    for i in xrange(N):
        
        # execute it on the robot
        plant.reset_state()
        apply_controller(plant,learner_params['params'], H, LQR)
    
if __name__ == '__main__':
    main()
    
