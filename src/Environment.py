
from numpy import linalg as la
import numpy as np
import random
import shapely

from Obstacle import shapelyObstacle
# from scipy.optimize import minimize
import qpsolvers
import cvxopt

class environment() :
    def __init__(self,outerbounds,obstacleData):
        # outerbounds:  a shapely polygon corresponding to the outer boundary of the workspace
        # obstacleData:   data provided in the yaml file for obstacles (Not unpacked)
        
        self.outerbounds=outerbounds
        self.obstacleData=obstacleData
        self.workspace=shapely.geometry.polygon.orient(outerbounds,1.0) # shapely polygon whose interior is the desired workspace             
        self.workspaceMatrix = np.array([[1, 0],[-1, 0],[0, 1],[0, -1]])
        self.workspaceCoefficients=np.array([[self.workspace.exterior.coords[1][0]],[self.workspace.exterior.coords[1][0]],[self.workspace.exterior.coords[1][0]],[self.workspace.exterior.coords[1][0]]])
    
    def radialNav(goal,state):    
        return goal-state
    
    def generateRndPoint(self, radius=None, refPt=None):  
        hitWorkspace=False
        while not hitWorkspace:
            if (radius is None) or (refPt is None):
                [xmin, ymin, xmax, ymax] = shapely.bounds(self.workspace)
                pt=np.array([[random.uniform(xmin,xmax)],[random.uniform(ymin,ymax)]])
                hitWorkspace=shapely.contains_xy(self.workspace, pt[0,0], pt[1,0])   
            else:   
                th=random.uniform(0,2*np.pi)
                rad=random.uniform(0,radius)
                pt=refPt+rad*np.array([[np.cos(th)],[np.sin(th)]])
                hitWorkspace=shapely.contains_xy(self.workspace, pt[0,0], pt[1,0])

        return pt

class sphereworldEnv(environment):

    def __init__(self,outerbounds,obstacleData):
        super().__init__(outerbounds,obstacleData)
        self.obstacleCenters=np.array(self.obstacleData['sphereWorld']['obsCenter'])
        self.obstacleRadii=np.array(self.obstacleData['sphereWorld']['obsRadii'])
        self.obstacleNum=len(self.obstacleRadii)
        for ii in range(len(self.obstacleRadii)):
            center=self.obstacleCenters[ii]
            radius=self.obstacleRadii[ii]
            self.workspace=shapely.difference(self.workspace,shapely.geometry.polygon.orient(shapelyObstacle.spawnSphere(center,radius),1.0))
    
    def navfSphere(self,state,goal):
        self.A=np.vstack((self.workspaceMatrix,self.safetyMatrix(state)))
        self.bb=np.vstack((self.workspaceCoefficients-(self.workspaceMatrix@goal).reshape(len(self.workspaceMatrix),1),self.safetyCoeffs(goal,state)))
        # result=qpsolvers.solve_qp(np.eye(2),np.zeros((2,1)),self.A,self.bb,solver='quadprog')
        res=cvxopt.solvers.qp(np.eye(2),np.zeros((2,1)),self.A,self.bb)
        if 'optimal' not in res['status']:
            return None
        result= np.array(res['x'])
        # result = minimize(self.objectiveFun, x0, constraints=constraints, options=options, method='SLSQP')
        return goal+result.reshape((2,1))-state

    def safetyMatrix(self,state):
        # Computes the coefficient matrix describing the safe polytope at the point z
        m=np.zeros((self.obstacleNum,2))
        for i in range(self.obstacleNum):
            m[i,:]=self.obstacleCenters[i]-state.reshape((1,2))
        return m
    def obstacleDist(self,state):
        # Computes the column vector of distances of z to the obstacle centers
        c=np.zeros((self.obstacleNum,1))
        for i in range(self.obstacleNum):
            c[i]=la.norm(state-self.obstacleCenters[i].T)
        return c     
    def safetyCoeffs(self,goal,state):
        # input should be column vectors
        b=np.zeros((self.obstacleNum,1))
        dists=self.obstacleDist(state)
        cons=self.safetyMatrix(state)
        b=b+0.5*(dists*dists-self.obstacleRadii.reshape(self.obstacleNum,1)*dists)+cons@(state-goal.reshape((2,1))) 
        return b


class polygonEnv(environment):    
    def __init__(self):
        obs=self.obstacleData['rectangle']
        obsNum=int(len(obs)/4)
        k=0        
        for ii in range(obsNum):
            faa=obs[k:k+4]
            self.workspace=shapely.difference(self.workspace,shapely.geometry.polygon.orient(shapelyObstacle.spawnPoly(faa),1.0))
            k += 4
class starWorldEnv(environment):
    def __init__(self):
        return