#/bin/bash python3

#System imports - not always used but necessary sometimes
import sys
import os
import math
from numpy import linalg as la
import numpy as np
import random
from scipy.sparse.csgraph import depth_first_order
import universal

class Agent:
    def __init__(self,name,env,network,task,pos):
        self.name=name
        self.env=env
        self.network=network
        self.pos=pos
        self.neighbors=network.neighbors(name)
        self.task=task

        
    def navf(self,pos):
        return self.env.navfSphere(pos,self.pos)

    def pollNeighborsPositions(self):
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
                navxi=self.network.tension_func(la.norm(relpos))*(relpos.T@relpos)[0][0]/(0.+(relpos.T@navvec)[0][0])
                pnpSummand=pnpSummand+navxi*navvec

        targ=self.task['target']
        if targ is None:
            controlInput=controlInput+pnpSummand
        else:
            controlInput=controlInput+pnpSummand+self.network.leaderGain*self.navf(targ)
        return controlInput
    
class sphereAgent(Agent):
    def __init__(self):

        return 
    
    



    

    











