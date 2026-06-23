#/bin/bash python3

import sys
import os
import math
import matplotlib.patches as patches
from numpy import linalg as la
import numpy as np
import random
# from scipy.sparse.csgraph import depth_first_order
from states import State
from states import State2ndOrder
import universal as uv
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from tf_transformations import quaternion_from_euler

# Future goal: Design agent class as a "parent" with minimal functionality, and re-structure for a fully-actuated and a unicycle agent separately.
skewJ = np.array([[0, -1],[1,0]]) # If you need it outside the file, write agent.skewJ




class Agent():
    def __init__(self,name,env,network,task,state):
        # super().__init__('baseAgent_node')
        self.name=name
        self.env=env
        self.network=network
        self.state=state # 6/9/2026 State will always have an attribute called "q" indicating position. It could have others too.
        self.neighbors=network.neighbors(name)
        self.task=task
        # UNCOMMENT THIS WHEN READY; IT IS INTENDED TO STAY HERE
        # self.visual=patches.Circle(
        #         uv.col2tup(self.agents[name].pos),
        #         radius=0.2,
        #         label=name,
        #         color='purple' if name in self.leaders else 'orange',
        #         animated=True,
        #         )

        # self.addEgde=self.create_service(AddEdge,'AddEdge',)
        # #Define parameters here
        # self.declare_parameter('x_init', self.pos[0])
        # self.declare_parameter('y_init', self.pos[1])
        # self.declare_parameter('name', self.name)
        
    def own_state(self,networkState=None): # returns the agent's state, or, if a networkState is provided, then returns the agent's component in it.
        if networkState is None:
            return self.state
        else:
            a,b=self.network.stateVectorInfo[self.name]
            return networkState[a:b,0]

    def pollNeighborsStates(self):
        if self.neighbors==None:
            return None
        else:
            return {name:self.network.reportState(name) for name in self.neighbors}
            # if networkState is None: # If no networkstate given, return dictionary of the states of all agents 
            #     return {name:self.network.reportState(name) for name in self.neighbors}
            # else: # If networkstate is given, return dictionary of states of all agents according to networkState vector
            #     d={}
            #     for name in self.neighbors:
            #         a,b=self.network.stateVectorInfo[name]
            #         d[name]=networkState[a:b,0]
            #     return d
    
    def navf(self,goal,inputState=None):
        if inputState is None:
            return self.env.nav(goal,self.state)
        else:
            return self.env.nav(goal,inputState)

    def translatePos(self,vec):
        self.state.q=self.state.q+vec # Think about this & state/2ndorder class
    
    def dynamics(self,controlInput,inputState=None): # DWR 6/17/2026: I think this is supposed to give a route to the controller computing for each agent type
        if inputState is None:
            inputState=self.state # Workaround for default value
        #some computation here using self.state and controlInput 
        return 0. # 0 is default dynamics value

    def computeController(self):
        #controller=self.navf(self.goal,self.state.q)
        controller=np.zeros(self.state.q.size) # Default controller is null
        return controller

    def clientOutputRobot(self): # For robot
        return self.computeController(self)
    
    def clientOutputSim(self,networkState): # for ODE simulation, networkState is a column np.array vector
        return self.dynamics(self.computeController(networkState),self.own_state(networkState))
        
    
class fullyActuatedAgent(Agent):
    def __init__(self,name,env,network,task,state):
        super().__init__(name,env,network,task,state)
        self.state=State(np.array(state['q']).T)
    
    def computeController(self,virtualState=None):  # See algorithm 4.2 in thesis for more info.
        states=self.pollNeighborsStates() # Obtain a list of neighbors' states and reduce them to positions
        if virtualState is None:
            positions={name:states[name].pos() for name in states}
        else:
            positions={}
            for name in states:
                a,b=self.network.stateVectorInfo[name]
                positions[name]=states[name].pos(virtualState[a:b,0])

        my_pos=self.own_state(virtualState)
        # Prepare "empty" control input vector
        controlInput=np.zeros_like(my_pos)
        # Prepare "empty" network interaction component
        pnpSummand=np.zeros_like(my_pos)
        # Accumulating the network interaction component
        if self.task['KeepUpQ']:
            for name in self.neighbors: # 6/9/2026: Consider rewriting to iterate through the entries of the positions dictionary
                navvec=self.navf(positions[name],my_pos)
                relpos=positions[name]-my_pos # 
                navxi=self.network.tension_func(la.norm(relpos))*(la.norm(relpos)**2)/(0.+(relpos.T@navvec))
                pnpSummand=pnpSummand+navxi*navvec

        # Calculate the navigation-to-goal component
        if 'Target' in self.task:
            #targ=np.matrix(self.task['Target'],shape=np.shape(my_pos)) # Old version from 6/22/2026
            targ=np.matrix(np.reshape(self.network.networkInfo['networkInfo']['networkTask']['Goals'][self.task['Target']],shape=np.shape(my_pos)))
            controlInput=controlInput+self.network.leaderGain*self.navf(State(targ.T),State(np.matrix(my_pos).T)) # nav requires State objects from states.py. Will this way cause lag?

        # Combine the target component with the network interaction component        
        controlInput=controlInput+pnpSummand
        return controlInput
    
    def dynamics(self,controlInput,inputState=None): # implementation of xdot=f(x,u)=u
        if inputState is None:
            inputState=self.state
        return controlInput
    

class unicycleAgent(Agent): #Will use 2nd order state from states.py
    def __init__(self,name,env,network,task,state):
        super().__init__(name,env,network,task,state)
        self.state=State2ndOrder(state)
        # self.pose=yaw
    def nav(self,goal):
        return self.env.nav(goal,self.pos)
    
    def pushField(self,pos):
        return self.env.pushField(self.pos,pos) # Not yet implemented
    
    def yawControl(self,goal):
        return self.beta*np.inner(self.navSphere(goal),skewJ*self.pose)
    
    def translatePos(self,vec):
        self.pos=self.pos+vec

    def eta(dist): # Dist is a scalar for distance from the obstacle
        epsilon = 1 # Dummy number
        return np.exp(-1/(epsilon**2 - dist))/np.exp(-1/(epsilon**2)) # Epsilon is a global not yet implemented, for pushaway field width

    def alpha(z):
        return 1 # dummy number
    
    def beta(z):
        return 1 # dummy number

    def Unicycle(self): # controller for robots, not the program.
        if not(self.task['keepUpQ']):
            raise TypeError("The unicycle code does not yet support multi-agent systems.")
            # Leader agents have keepUpQ set to false
        else:
            navvec=self.nav(self.pos) # This is n from the paper
            pushvec=self.pushField(self.pos) # This is m from the paper
            delta = 1 # dummy number
            z = 1 # dummy number
            velocity = (1-self.eta(delta**2))*self.alpha(z)*np.inner(navvec,self.p) + self.eta(delta**2)*np.inner(pushvec,self.p)
            angVelocity = (1-self.eta(delta**2))*self.beta(z)*np.inner(navvec,skewJ*self.p) + self.eta(delta**2)*np.inner(pushvec,skewJ*self.p)
        return (velocity,angVelocity)
            # Notes
            #   veloc and angveloc come from the Unicycle PnP paper's feedback controller section on pg 1
            #   alpha, beta, and pushvec are not implemented yet.
            #   Currently the algorithm is written for a single-agent situation, as I couldn't identify much info pertaining to multi-
            #       agent situations in the Unicycle PnP paper.

    # def clientOutput(self): # May want to delete this, since there's clientoutputrobot and sim in the parent class
    #     controlInputVeloc=velocity*self.p
    #     controlInputAngle=angVelocity*skewJ*self.p
    #     return 
    
    # testing notes:
    #   Set dummy values for eta, alpha, and beta




    

    











