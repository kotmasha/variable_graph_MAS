import numpy as np

class State(): # Class for representing state vectors
    def __init__(self,pos,): # DWR 6/22/2026: Change all places where State and 2ndOrder are used to reflect new input of a single vector
        self.q=np.matrix(pos).reshape(-1,1)
    
    def size(self):
        return self.q.size
    
    def flatten(self):
        return self.q # for now
    
    def update(self,flattened_new_state): # updates the state based on a flattened vector representation
        self.q=flattened_new_state

    def pos(self,vec=None):
        if vec is None:
            return self.q
        else:
            return vec

class State2ndOrder(State):
    def __init__(self,stateVec): # stateVec is numpy column matrix with even number of entries
        s=stateVec.size
        super().__init__(stateVec[0:s/2,0])
        self.p=np.matrix(stateVec[s/2:s,0]).reshape(-1,1)
    
    def flatten(self):#Goal: put q and p together as a single column vector and return it
        return np.concatenate((self.q,self.p)).T # DWR note: I am not sure whether this is a horizontal or vertical vector. Hopefully there's a way to check.

    def size(self): 
        return self.flatten().size
    
    def update(self,flattened_new_state): # updates the state based on a flattened vector representation, which must be a numpy column matrix
        s=self.size(flattened_new_state)
        self.q=flattened_new_state[0:s/2,0]
        self.p=flattened_new_state[s/2:s,0]

    def pos(self,vec=None): # returns the position component of the state/of the input vector vec; vec must be a numpy column matrix
        if vec is None:
            return self.q
        else:
            return vec[0:len(vec)/2,0]