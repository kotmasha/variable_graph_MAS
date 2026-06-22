import numpy as np

class State(): # Class for representing state vectors
    def __init__(self,pos,):
        self.q=np.array(pos)
    
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
    def __init__(self,q,p): # q and p should be the same size
        super().__init__(q)
        self.p=np.array(p)
    
    def flatten(self):#Goal: put q and p together as a single column vector and return it
        return np.concatenate((self.q,self.p)).T # DWR note: I am not sure whether this is a horizontal or vertical vector. Hopefully there's a way to check.

    def size(self): 
        return self.flatten().size
    
    def update(self,flattened_new_state): # updates the state based on a flattened vector representation
        s=self.size(flattened_new_state)
        self.q=flattened_new_state[0:s/2-1,0]
        self.p=flattened_new_state[s/2,s-1]

    def pos(self,vec=None):
        if vec is None:
            return self.q
        else:
            return vec[0:len(vec)/2]