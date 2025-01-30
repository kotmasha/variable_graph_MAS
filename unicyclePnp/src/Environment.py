
from numpy import linalg as la
import numpy as np
import random
import shapely
import shapely.plotting
import shapely.geometry as geom
from shapely.ops import unary_union

from Obstacle import shapelyObstacle
# from scipy.optimize import minimize
import qpsolvers
# import cvxopt
import quadprog
import sys
class environment() :
    def __init__(self,outerbounds,obstacleData):
        # outerbounds:  a shapely polygon corresponding to the outer boundary of the workspace
        # obstacleData:   data provided in the yaml file for obstacles (Not unpacked)
        
        self.outerbounds=outerbounds
        self.obstacleData=obstacleData
        self.workspace=shapely.geometry.polygon.orient(outerbounds,1.0) # shapely polygon whose interior is the desired workspace             
        self.workspaceMatrix = np.array([[1, 0],[-1, 0],[0, 1],[0, -1]])
        self.wMax=int(self.workspace.exterior.coords[1][0])
        self.workspaceCoefficients=np.array([[self.wMax],[self.wMax],[self.wMax],[self.wMax]])
        self.stateDim = 2


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
        self.workspaceFinal=self.workspace
        # shapely.plotting.plot_polygon(self.workspace)

    def navfSphere(self,state,goal):
        self.A=np.vstack((self.workspaceMatrix,self.safetyMatrix(state)))
        self.bb=np.vstack((self.workspaceCoefficients-(self.workspaceMatrix@goal).reshape(len(self.workspaceMatrix),1),self.safetyCoefficients(goal,state)))
        result=qpsolvers.solve_qp(np.eye(2),np.zeros((2,1)),self.A,self.bb,solver='piqp')
        print(result)
        if result is None:
            result = np.zeros((2,1))
        return goal+result.reshape((2,1))-state
    
    def safetyMatrix(self,state):
        # Computes the coefficient matrix describing the safe polytope at the point z

        m=np.zeros((self.obstacleNum,self.stateDim))
        for i in range(self.obstacleNum):
            m[i,:]=self.obstacleCenters[i]-state.T
            # pdb.set_trace()
        return m

    def obstacleDist(self,state):
        # Computes the column vector of distances of z to the obstacle centers
        c=np.zeros((self.obstacleNum,1))
        for i in range(self.obstacleNum):
            col = state - self.obstacleCenters[i,:].reshape((2,1))
            c[i,:] = np.sqrt(col.T @ col)
            # c[i]=la.norm(state-self.obstacleCenters[i].T)
        return c     
    
    def safetyCoefficients(self,goal,state):
        # input should be column vectors
        b=np.zeros((self.obstacleNum,1))
        dists=self.obstacleDist(state)
        cons=self.safetyMatrix(state)
        b=b+0.5*(dists*dists-self.obstacleRadii.reshape(-1,1)*dists)+cons @ (state-goal.reshape((2,1))) 
        return b
    def inCircle(self,idx,idy,oc,oR):
        return (idx-oc[0])**2 + (idy-oc[1])**2 <= oR**2
    
    def isSafe(self,idx,idy):
        count = 0
        for i in range(self.obstacleNum): 
            oc = self.obstacleCenters[i]
            oR = self.obstacleRadii[i]
            if self.inCircle(idx,idy,oc,oR):
                count = count + 1 # Point is inside an obstacle, not safe
        if count > 0:
            return False
        else:
            return True
        
    def plotObstacles(self,viz):
        for i in range(self.obstacleNum):
            oc = shapely.Point(self.obstacleCenters[i])
            oR = self.obstacleRadii[i]
            circ = oc.buffer(oR)
            x, y = circ.exterior.xy
            viz.plot(x, y, color='black')
            viz.fill(x, y, color='red',alpha=0.99 )
            if i ==0:
                viz.text(circ.centroid.x,circ.centroid.y,"Obstacles",fontsize=20,color='white',ha='center',va='center')





class starworldEnv(environment):
    def __init__(self,outerbounds,obstacleData):
        super().__init__(outerbounds,obstacleData)
        
        self.obstacleNumPoints=360
        self.obstacleNum = self.obstacleData['starWorld']['obsNum']
        self.obstacleCenters=np.ones((self.obstacleNum,self.stateDim))
        self.obstacleRadii=np.ones((self.obstacleNum))
        
        for ii in range(self.obstacleNum):
            theta=2*np.pi*(ii-1)/self.obstacleNum
            self.obstacleCenters[ii,:]=np.array([8*np.cos(theta),8*np.sin(theta)])
            self.obstacleRadii[ii]=ii+0.5
            if ii ==2:
                self.obstacleRadii[ii]=ii-0.5
        # print(self.obstacleCenters,self.obstacleRadii)
        self.obstacleClearance=8*np.array((self.obstacleData['starWorld']['obsClearance']))
        print(self.obstacleClearance)
        angles = np.linspace(0, 2 * np.pi, self.obstacleNumPoints, endpoint=False)
        angles = np.append(angles, 0) 
        obstacleBufferList=[]

        for ii in range(self.obstacleNum):
            oc = self.obstacleCenters[ii, :]
            radii = self.barrierCurve(ii, angles)
            
            # Create star-shaped obstacle polygon
            x, y = radii * np.cos(angles), radii * np.sin(angles)
            x += oc[0]
            y += oc[1]
            obstacle = shapely.Polygon(zip(x, y)).buffer(0)
            
            # Subtract obstacle from workspace
            self.workspace = shapely.difference(self.workspace, obstacle)
            
            # Create buffered region around the obstacle
            radii_buffer = np.sqrt(self.barrierCurve(ii, angles)**2 + self.obstacleClearance[ii]**2)
            x_buffer, y_buffer = radii_buffer * np.cos(angles), radii_buffer * np.sin(angles)
            x_buffer += oc[0]
            y_buffer += oc[1]
            
            # Create obstacle buffer polygon
            obstacle_buffer = shapely.Polygon(zip(x_buffer, y_buffer)).buffer(0)  # Buffer set to 0 to create a polygon only
            # self.workspace = shapely.difference(self.workspace, obstacle_buffer)
            obstacleBufferList.append(obstacle_buffer)
            # Intersect with workspace
            # obstacle_buffer_set = obstacle_buffer_set
            
            # Add to the union of buffers
            # obstacleBuffer = unary_union([obstacleBuffer, obstacle_buffer])


    def plotObstacles(self,viz):
        angles = np.linspace(0, 2 * np.pi, self.obstacleNumPoints, endpoint=False)
        angles = np.append(angles, 0) 
        obstacleBufferList=[]

        for ii in range(self.obstacleNum):
            oc = self.obstacleCenters[ii, :]


            # Create buffered region around the obstacle
            radii_buffer = np.sqrt(self.barrierCurve(ii, angles)**2 + self.obstacleClearance[ii]**2)
            x_buffer, y_buffer = radii_buffer * np.cos(angles), radii_buffer * np.sin(angles)
            x_buffer += oc[0]
            y_buffer += oc[1]
            
            # Create obstacle buffer polygon
            obstacle_buffer = shapely.Polygon(zip(x_buffer, y_buffer)).buffer(0)  # Buffer set to 0 to create a polygon only
            # Plot the obstacle buffer with alpha=0.7
            viz.fill(x_buffer, y_buffer, alpha=0.7, facecolor='green')
            radii = self.barrierCurve(ii, angles)
            
            # Create star-shaped obstacle polygon
            x, y = radii * np.cos(angles), radii * np.sin(angles)
            x += oc[0]
            y += oc[1]
            obstacle = shapely.Polygon(zip(x, y)).buffer(0)
            
            # Subtract obstacle from workspace
            self.workspace = shapely.difference(self.workspace, obstacle)
            viz.fill(x, y, color='red',alpha=0.99)
            # viz.plot(obstacle_buffer, ax=viz, alpha=0.7, facecolor='green')

        
        
    def navfSphere(self,state,goal):
        self.A=np.vstack((self.workspaceMatrix,self.safetyMatrix(state)))
        self.bb=np.vstack((self.workspaceCoefficients-(self.workspaceMatrix@goal).reshape(len(self.workspaceMatrix),1),self.safetyCoefficients(goal,state)))
        result=qpsolvers.solve_qp(np.eye(2),np.zeros((2,1)),self.A,self.bb,solver='piqp')
        
        return goal+result.reshape((2,1))-state
    
    def safetyMatrix(self,state):
        # Computes the coefficient matrix describing the safe polytope at the point z

        m=np.zeros((self.obstacleNum,self.stateDim))
        for i in range(self.obstacleNum):
            m[i,:]=self.obstacleCenters[i]-state.T

            # pdb.set_trace()
        return m

    def obstacleDist(self,state):
        # Computes the column vector of distances of z to the obstacle centers
        c=np.zeros((self.obstacleNum,1))
        for i in range(self.obstacleNum):
            col = state - self.obstacleCenters[i,:].reshape((2,1))
            c[i,:] = np.sqrt(col.T @ col)
            # c[i]=la.norm(state-self.obstacleCenters[i].T)
        return c     
    
    def safetyCoefficients(self,goal,state):
        # input should be column vectors
        b=np.zeros((self.obstacleNum,1))
        dists=self.obstacleDist(state)
        cons=self.safetyMatrix(state)
        b=b+0.5*(dists*dists-self.obstacleRadii.reshape(-1,1)*dists)+cons @ (state-goal.reshape((2,1))) 
        return b
    
    # def navfStar(self,state,goal):
    #     # ensure state,goal are coloumn vecs
    #     # print(state,goal)
    #     newG=self.wksp2sph(goal)
    #     newState=self.wksp2sph(state)
    #     # A = np.linalg.det(self.wksp2sphDeriv(state))
    #     return np.linalg.inv(self.wksp2sphDeriv(state))*self.navfSphere(newState,newG)
    
    def navfStar(self, state, goal):
        # Ensure state and goal are column vectors
        state = state.reshape((2, 1))
        goal = goal.reshape((2, 1))

        # Transform state and goal to sphere coordinates
        newG = self.wksp2sph(goal)
        newState = self.wksp2sph(state)

        # Calculate navigation function in sphere coordinates
        nav_sphere = self.navfSphere(newState, newG)

        # Calculate the Jacobian of the workspace-to-sphere transformation
        J = self.wksp2sphDeriv(state)

        # Transform the navigation vector back to workspace coordinates
        nav_star = np.linalg.solve(J, nav_sphere)
        print(f'nav_star: {nav_star}')
        return nav_star

    def wksp2sph(self,p):
        stitches=self.stitchingMap(p)
        q=stitches[self.obstacleNum,0]*p
        # print(f'i am0:{q}')
        for ii in range(self.obstacleNum):
            oc=self.obstacleCenters[ii,:].reshape((2,1))
            rad=self.obstacleRadii[ii]
            stitch = stitches[ii,0]
            # print(p,oc)
            relpos=p-oc
            # print(relpos)
            relnorm=la.norm(relpos)
            q=q+stitch*(oc+rad*relpos/relnorm)
        # print(f'i am1:{q}')
        return q
    
    def wksp2sphDeriv(self,p):
        stitches=self.stitchingMap(p)
        print('in function wksp2deriv',stitches)
        stitchesDeriv=self.smDeriv(p)
        D=stitches[self.obstacleNum]*np.eye(self.stateDim)

        for ii in range(self.obstacleNum):
            oc=self.obstacleCenters[ii,:].T
            rad=self.obstacleRadii[ii]
            stitch = stitches[ii,0]
            relpos=p-oc
            relnorm=la.norm(relpos)
            D=D+(stitch*rad/relnorm)*np.eye(self.stateDim)
            D=D+((rad/relnorm)-1)*relpos*stitchesDeriv[ii,:]
            D=D-stitch*rad*(relnorm**-3)*(relpos*relpos.T)
        return D
    
    def stich(self,i,x):
        epsilon=self.obstacleClearance[i]
        out = self.bump(epsilon-x)/self.bump(epsilon)
        return out
    def stitchDeriv(self,i,x):
        epsilon=self.obstacleClearance[i]
        print(epsilon,x,epsilon-x)
        print('in stich deriv',self.bumpDeriv(epsilon-x),self.bump(epsilon))
        return -self.bumpDeriv(epsilon-x)/self.bump(epsilon)
    
    def stitchingMap(self,p):
        v=np.zeros([self.obstacleNum+1,1])
        v[self.obstacleNum,0]=1
        # print(f'p:{p}')
        for ii in range(self.obstacleNum):
            v[ii,0]=v[ii,0]+self.stich(ii,self.barrier(ii,p))
            # print(v,ii)
            v[self.obstacleNum]=v[self.obstacleNum,0]-v[ii,0]

        return v
    
    def smDeriv(self,p):
        out=np.zeros((self.obstacleNum+1,self.stateDim))
        for ii in range(self.obstacleNum):
            # print()
            print('in function smDeriv',self.stitchDeriv(ii,self.barrier(ii,p)),self.barrierDeriv(ii,p))
            if self.stitchDeriv(ii,self.barrier(ii,p)) == np.zeros((1,1)):
                out[ii,:]=np.zeros((1,2))
            else:

                out[ii,:]=(self.stitchDeriv(ii,self.barrier(ii,p))) @ self.barrierDeriv(ii,p)
        
        out[ii,:] = -np.sum(out)
        return out

    
    def barrier(self,i,p):

        # 
        obsC=self.obstacleCenters[i].reshape((2,1))
        # print(f'check if vert: {obsC} ')
        # print(f'p:{p}')
        relpos = p - obsC
        # print(relpos)
        rho= np.sqrt(relpos[0]**2 + relpos[1]**2)
        # print(f'rho:{rho}')
        theta=np.arctan2(relpos[1], relpos[0])
        r0 = self.barrierCurve(i,theta)
        # print(rho**2-r0**2)
        return rho**2-r0**2
    
    def barrierDeriv(self,i,p):

        obsC=self.obstacleCenters[i].reshape((2,1))
        # print(f'check if vert {obsC} ')
        obsR = self.obstacleRadii[i]
        relpos = p - obsC
        rho= np.sqrt(relpos[0]**2 + relpos[1]**2)
        theta=np.arctan2(relpos[1], relpos[0])
        u = np.array([np.cos(theta),np.sin(theta)])
        v = np.array([np.sin(theta),-np.cos(theta)])
        r0 = self.barrierCurve(i,theta)
        r1 = self.barrierCurveDeriv(i,theta)
        # print(2*(relpos+(r0*r1/rho)*v))
        return 2*(relpos+(r0*r1/rho)*v).reshape((1,2))
    
    def barrierCurve(self,i,th):
        obsR = self.obstacleRadii[i]
        if i ==0:
            nPeaks=3
            out=1.1*obsR+(1+np.sin(nPeaks*th))
            return out
        elif i==1:
            nPeaks=7
            out=1.1*obsR+(1+np.sin(nPeaks*th))
            return out
        elif i==2:
            offsetAngle=np.pi/6
            nPeaks=2
            out=1.1*obsR+2*(1+np.cos(nPeaks*(th+offsetAngle)))
            return out
        
    
    def barrierCurveDeriv(self,i,th):

        obsR = self.obstacleRadii[i]
        if i ==0:
            nPeaks=3
            out=nPeaks*np.cos(nPeaks*th)
        elif i==1:
            nPeaks=7
            out=nPeaks*np.cos(nPeaks*th)
        elif i==2:
            # offsetAngle=np.pi/6
            nPeaks=2
            out=-2*nPeaks*np.sin(nPeaks*(th))

        return out
    def bump(self,x):
        return self.cInftyBump(x)
    
    def bumpDeriv(self,x):
        # print(f'i am x:{x}')
        return self.cInftyBumpDeriv(x)
    
    def cInftyBump(self, x):
        """C^\infty bump function."""
        # print("in cinfbump",x)
        if x <= 0:
            return np.zeros((1,1))
        else:
            # print('waatt',np.exp(-1 / x))
            return np.exp(-1 / x)
    
    def cInftyBumpDeriv(self, x):
        """Derivative of the C^\infty bump function."""
        if x <= 0:
            return np.zeros((1,1))
        else:
            return np.exp(-1 / x) * (x ** -2)
    
    def cOneBump(self, x):
        """C^1 bump function."""
        if x <= 0:
            return 0
        else:
            return x ** 2
    
    def cOneBumpDeriv(self, x):
        """Derivative of C^1 bump function."""
        if x <= 0:
            return 0
        else:
            return 2 * x
    
    def cTwoBump(self, x):
        """C^1 bump function."""
        if x <= 0:
            return 0
        else:
            return x ** 3
    
    def cTwoBumpDeriv(self, x):
        """Derivative of C^1 bump function."""
        if x <= 0:
            return 0
        else:
            return 3 * x ** 2






class polygonEnv(environment):    
    def __init__(self):
        obs=self.obstacleData['rectangle']
        obsNum=int(len(obs)/4)
        k=0        
        for ii in range(obsNum):
            faa=obs[k:k+4]
            self.workspace=shapely.difference(self.workspace,shapely.geometry.polygon.orient(shapelyObstacle.spawnPoly(faa),1.0))
            k += 4

    # \nRadius of\ncommunication=3m