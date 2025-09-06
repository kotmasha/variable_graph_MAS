import sys
import os
import math
from numpy import linalg as la
import numpy as np
import random
import quadprog
import cvxopt
from scipy.optimize import minimize,LinearConstraint
class navigation:
    def __init__(self,wkspCoeffs,wkspMatrix,obstacleNum,obstacleRadius,obstacleCenters): 

        self.wkspMatrix=wkspMatrix
        self.wkspCoeffs=wkspCoeffs
        self.obstacleNum=obstacleNum
        self.obstacleRadius=obstacleRadius
        self.obstacleCenters=obstacleCenters
        




    def navfSph(self,state,goal):
        #M is workspace matrix
        #C is 
        # A is the coeff matrix of inequality 
        A = np.vstack((self.workspaceMatrix, self.safetyMatrix(state)))
        bb = np.vstack((self.workspaceCoefficients - self.workspaceMatrix @ goal, self.safetyCoefficients(goal,state)))
        constraints = ({'type': 'ineq', 'fun': lambda x: A.dot(x) - bb})
        x0=state-goal
        options={'disp':False}
        result = minimize(self.objective, x0, constraints=constraints, options=options, method='SLSQP')
        return goal+result.x-state
    
    def objective(x):
        return np.sum(x**2)
    
    def safetyCoefficients(self,goal,state):
        # input should be column vectors
        b=np.zeros((self.obstacleNum,1))
        dists=self.obsDist(state)
        cons=self.safetyMatrix(state)
        b=b+0.5*(dists*dists-self.obstacleRadius*dists)+cons @ (state-goal) 
        return b
    
    def safetyMatrix(self,state):
        # Computes the coefficient matrix describing the safe polytope at the point z
        m=np.zeros((self.obstacleNum,1))
        
        for i in len(self.obstacleNum):
            m[i,:]=self.obstacleCenters[i,:]-state.T
        

    def obsDist(self,state):
        # Computes the column vector of distances of z to the obstacle centers
        c=np.zeros((self.obstacleNum,1))
        for i in len(self.obstacleNum):
            c[i,1]=la.norm(state-self.obstacleCenters[i,:].T)
        return c
    

