# Assignment2: Cart-pole balancing 

## Executing the Code

To run the PID controller open 'cartpole_learn.py' and change the policy to policyfn 
To run the LQR controller open 'cartpole_learn.py' and change the policy to LQR

The policy can be changed by passing policyfn or LQR as an arguement in the applycontrol method. 

## Getting started with the provided code 

- Ensure that you have installed python, numpy matplotlib and scipy
- Run the provided example with `python cartpole_learn.py`
  - A GUI window should appear showing the carpole falling over
  - The code should print the error per episode to terminal standard output

## Starting to write your own code

You only have to add code to one file: src/cartpole_learn.py. Except for parameter changes to the simulator that you might try, all the code can be placed in the policyfn method.

- This method takes x, the state of the cartpole which is expressed in 5 dimensions:
  - The cart position, x, in x[0]
  - The cart linear velocity, x_dot, in x[1]
  - The cart angular velocity, theta_dot, in x[2]
  - The sin of the cart's angle, sin(theta), in x[3]
  - The cos of the cart's angle, cos(theta), in x[4]
- We use the cos/sin representation to prevent problems with angle wrap-around
  



