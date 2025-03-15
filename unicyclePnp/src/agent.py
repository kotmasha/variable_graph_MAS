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
        self.pos=self.pos-vec
        # print(self.pos)
    def translatePose(self,vec):
        self.pose=vec

    # def navf(self,pos):
    #     return self.env.polyNav(self.pos,pos)
    def navf(self,pos):
        return self.env.navfSphere(self.pos,pos)

    def alpha1(self,pos,pose):
        
        num=pose.T@self.navf(self.target)*pose.T@self.env.distanceGradient(pos)
        return num
    
    def alpha2(self,pos,pose):
        nub=(self.skewJ@pose).T@self.navf(self.target)*pose.T@self.env.distanceGradient(pos)
        return nub
    
    def forward_control_input(self):
        v=self.alpha1(self.pos,self.pose)*self.pose.T@self.navf(self.target)+self.alpha2(self.pos,self.pose)*(self.skewJ@self.pose).T@self.navf(self.target)
        return v*self.pose

    def angular_control_input(self):
        w=-self.alpha2(self.pos, self.pose)*self.pose.T@self.navf(self.target)+self.alpha1(self.pos,self.pose)*(self.skewJ@self.pose)
        return w*(self.skewJ@self.pose)
    
    def unitPose(self,pose):
        """
        Convert a 2D column vector (numpy array with shape (n,1)) to a unit vector.
        
        Args:
            pose: A numpy array of shape (n,1) representing a column vector
            
        Returns:
            A numpy array of the same shape, normalized to unit length
        """
        # Ensure pose is a numpy array
        if not isinstance(pose, np.ndarray):
            pose = np.array(pose)
        
        # Handle different shapes
        if pose.ndim == 2:
            # If it's a 2D array (column vector), flatten it for calculations
            pose_flat = pose.flatten()
        else:
            # If it's already 1D, use it directly
            pose_flat = pose
        
        # Calculate the vector's magnitude (length) using Pythagorean theorem
        magnitude = math.sqrt(sum(x**2 for x in pose_flat))
        
        # Check for zero vector to avoid division by zero
        if magnitude == 0:
            return np.zeros_like(pose)  # Return zero vector of same shape
        
        # Normalize the vector
        if pose.ndim == 2:
            # For 2D array, return the same shape
            unit_vector = np.array([[x / magnitude] for x in pose_flat]).reshape(pose.shape)
        else:
            # For 1D array, return 1D
            unit_vector = np.array([x / magnitude for x in pose_flat])
        
        return unit_vector










