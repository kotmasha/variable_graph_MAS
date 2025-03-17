#/bin/bash python3

import sys
import os
import math
from numpy import linalg as la
import numpy as np
import random
from scipy.sparse.csgraph import depth_first_order
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
    def translatePose(self,vec):
        self.pose=self.pose+vec

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


class unicycleAgent(Agent):
    def __init__(self,name,env,network,task,pos):
        super().__init__(name,env,network,task,pos)
        rawpos=pos[:2,0]
        rawpose=pos[-2:,0]
        self.pos = np.array([rawpos[0], rawpos[1]]).reshape(2, 1)        
        self.pose = np.array([rawpose[0], rawpose[1]]).reshape(2, 1)
        # self.pose=self.unitPose(self.notunitpose)
        self.skewJ = np.array([[0, -1],[1,0]])
        self.target=self.task['target'].reshape((2,1))
    def translatePos(self,vec):
        self.pos=self.pos+vec
        # print(self.pos)
    def translatePose(self,vec):
        self.pose=self.pose+vec

    def navf(self,pos):
        return self.env.navfSphere(self.pos,pos)

    def alpha1(self,pos,pose):
        num=pose.T@self.navf(self.target)*pose.T@self.nablaQDel(pos)
        return num
    
    def alpha2(self,pos,pose):
        nub=(self.skewJ@pose).T@self.navf(self.target)*pose.T@self.nablaQDel(pos)
        return nub
    def nablaQDel(self, pos):
        delta = self.env.nearestUnsafePoint(pos)
        if delta is None:
            return None
        else:
            difference = pos - delta
            print(f"my pos, nearest point and difference: {self.pos,delta,difference}")
            norm = np.linalg.norm(difference)
            if norm != 0:  
                normalized_difference = difference / norm
            else:
                print('wrong somewhere')
                normalized_difference = difference  
            return normalized_difference
    def forward_control_input(self):
        # print(f"alpha1(self.pos, self.pose): {self.alpha1(self.pos, self.pose)}")
        # print(f"self.pose.T @ self.navf(self.target): {self.pose.T @ self.navf(self.target)}")
        # print(f"alpha2(self.pos, self.pose): {self.alpha2(self.pos, self.pose)}")
        # print(f"(self.skewJ @ self.pose).T @ self.navf(self.target): {(self.skewJ @ self.pose).T @ self.navf(self.target)}")
        
        v = self.alpha1(self.pos, self.pose) * self.pose.T @ self.navf(self.target)
        # + self.alpha2(self.pos, self.pose) * (self.skewJ @ self.pose).T @ self.navf(self.target)
        # print(f"v (before scaling): {v}")
        
        result = v * self.pose
        # print(f"result (after scaling): {result}")
        
        return result

    def angular_control_input(self):
        # print(f"alpha2(self.pos, self.pose): {self.alpha2(self.pos, self.pose)}")
        # print(f"self.pose.T @ self.navf(self.target): {self.pose.T @ self.navf(self.target)}")
        # print(f"alpha1(self.pos, self.pose): {self.alpha1(self.pos, self.pose)}")
        # print(f"(self.skewJ @ self.pose): {(self.skewJ @ self.pose)}")
        
        w = self.alpha2(self.pos, self.pose) * (self.skewJ @ self.pose).T @ self.navf(self.target) 
        # + self.alpha1(self.pos, self.pose) * (self.skewJ @ self.pose)
        # print(f"w (before scaling): {w}")
        
        result = w * (self.skewJ @ self.pose)
        # print(f"result (after scaling): {result}")
        
        return result
    









