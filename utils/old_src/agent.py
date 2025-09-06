#/bin/bash python3

import sys
import os
import math
from numpy import linalg as la
import numpy as np
import random
# from scipy.sparse.csgraph import depth_first_order
import universal
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from tf_transformations import quaternion_from_euler



class Agent():
    def __init__(self,name,env,network,task,pos):
        # super().__init__('baseAgent_node')
        self.name=name
        self.env=env
        self.network=network
        self.pos=pos
        self.neighbors=network.neighbors(name)
        self.task=task

        # self.addEgde=self.create_service(AddEdge,'AddEdge',)
        # #Define parameters here
        # self.declare_parameter('x_init', self.pos[0])
        # self.declare_parameter('y_init', self.pos[1])
        # self.declare_parameter('name', self.name)
        
    def navf(self,pos):
        return self.env.navfSphere(self.pos,pos)

    def pollNeighborsPositions(self):
        if self.neighbors==None:
            return None
        else:

            return {name:self.network.reportPosition(name) for name in self.neighbors}
    
    def translatePos(self,vec):
        self.pos=self.pos+vec

    def pnp(self):        
        positions=self.pollNeighborsPositions()
        controlInput=np.zeros((2,1))
        pnpSummand=np.zeros((2,1))
        if self.task['keepUpQ']:
            for name in self.neighbors:
                navvec=self.navf(positions[name])
                relpos=positions[name]-self.pos
                navxi=self.network.tension_func(la.norm(relpos))*(la.norm(relpos)**2)/(0.+(relpos.T@navvec))
                # print(self.name,self.network.tension_func(la.norm(relpos)),(la.norm(relpos)*la.norm(relpos)),(0.+(relpos.T@navvec)))
                pnpSummand=pnpSummand+navxi*navvec

        targ=self.task['target']
        if targ is None:
            controlInput=controlInput+pnpSummand
        else:
            targ=targ.reshape((2,1))
            controlInput=controlInput+self.network.leaderGain*self.navf(targ) 
        # print(controlInput)
        return controlInput
    def pnpUnicycle(self):
        # positions=self.pollNeighborsPositions()
        controlInput=np.zeros((2,1))
        pnpSummand=np.zeros((2,1))
class unicycleAgent(Agent):
    def __init__(self,name,env,network,task,pos,yaw):
        super().__init__(name,env,network,task,pos)
        self.pose=yaw
        self.skewJ = np.array([[0, -1],[1,0]])
    def navSphere(self,pos):
        return self.env.navfSphere(self.pos,pos)
    
    def yawControl(self,goal):
        return self.beta*np.inner(self.navSphere(goal),self.skewJ*self.pose)
    



    

    











