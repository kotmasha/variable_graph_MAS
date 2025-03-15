
from numpy import linalg as la
import numpy as np
import random
import shapely
import shapely.plotting
import shapely.geometry as geom
from shapely.ops import unary_union
from shapely.geometry import Polygon, Point
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
    def generateRandPointPose(self):
        min_x, min_y, max_x, max_y = self.outerbounds.bounds
        
        while True:
            x = np.random.uniform(min_x, max_x)
            y = np.random.uniform(min_y, max_y)
            point = shapely.geometry.Point(x, y)
            
            inside_obstacle = any(
                point.distance(shapely.geometry.Point(center)) < radius
                for center, radius in zip(self.obstacleCenters, self.obstacleRadii)
            )
            
            if not inside_obstacle and self.workspace.contains(point):
                return np.array([x, y, np.cos(theta := np.random.uniform(0, 2 * np.pi)), np.sin(theta)]).reshape((4,1))

    def navfSphere(self,state,goal):
        goal=goal.reshape((2,1))
        state=state.reshape((2,1))
        self.A=np.vstack((self.workspaceMatrix,self.safetyMatrix(state)))
        self.bb=np.vstack((self.workspaceCoefficients-(self.workspaceMatrix@goal).reshape(len(self.workspaceMatrix),1),self.safetyCoefficients(goal,state)))
        result=qpsolvers.solve_qp(np.eye(2),np.zeros((2,1)),self.A,self.bb,solver='piqp')
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
    def distanceGradient(self, state):
        state = np.asarray(state).reshape(2, 1)
        point = shapely.Point(float(state[0]), float(state[1]))
        
        min_dist = float('inf')
        gradient = np.zeros((2, 1))
        
        for i in range(self.obstacleNum):
            obstacle_center = self.obstacleCenters[i]
            radius = self.obstacleRadii[i]
            obstacle = shapely.Point(obstacle_center).buffer(radius)
            
            dist_to_obstacle = point.distance(obstacle)
            if shapely.contains(obstacle, point):
                dist_to_obstacle = -dist_to_obstacle
                
            vec_to_state = state - obstacle_center.reshape((2, 1))
            dist_to_center = np.linalg.norm(vec_to_state)
            
            if abs(dist_to_obstacle) < min_dist:
                min_dist = abs(dist_to_obstacle)
                gradient = vec_to_state / max(dist_to_center, 1e-10)
        
        if not shapely.contains(self.workspace, point):
            nearest_point = shapely.ops.nearest_points(point, self.workspace)[1]
            boundary_vec = np.array([[nearest_point.x - point.x], [nearest_point.y - point.y]])
            dist_to_boundary = np.linalg.norm(boundary_vec)
            
            if dist_to_boundary < min_dist:
                min_dist = dist_to_boundary
                gradient = boundary_vec / max(dist_to_boundary, 1e-10)
        else:
            boundary = shapely.boundary(self.workspace)
            nearest_point = shapely.ops.nearest_points(point, boundary)[1]
            boundary_vec = np.array([[point.x - nearest_point.x], [point.y - nearest_point.y]])
            dist_to_boundary = np.linalg.norm(boundary_vec)
            
            if dist_to_boundary < min_dist:
                min_dist = dist_to_boundary
                gradient = boundary_vec / max(dist_to_boundary, 1e-10)
        
        norm = np.linalg.norm(gradient)
        return gradient / max(norm, 1e-10)
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
        # print("in cinfbump",x)
        if x <= 0:
            return np.zeros((1,1))
        else:
            # print('waatt',np.exp(-1 / x))
            return np.exp(-1 / x)
    
    # def cInftyBumpDeriv(self, x):
    #     """Derivative of the C^\infty bump function."""
    #     if x <= 0:
    #         return np.zeros((1,1))
    #     else:
    #         return np.exp(-1 / x) * (x ** -2)
    
    def cOneBump(self, x):
        if x <= 0:
            return 0
        else:
            return x ** 2
    
    def cOneBumpDeriv(self, x):
        if x <= 0:
            return 0
        else:
            return 2 * x
    
    def cTwoBump(self, x):
        if x <= 0:
            return 0
        else:
            return x ** 3
    
    def cTwoBumpDeriv(self, x):
        if x <= 0:
            return 0
        else:
            return 3 * x ** 2

class polygonEnv(environment):    
    def __init__(self,outerbounds,obstacleData):
        super().__init__(outerbounds,obstacleData)
        recObs=self.obstacleData['rectangle']
        hexObs=self.obstacleData['hex']
        recObsNum=int(len(recObs)/4)
        hexObsNum=int(len(hexObs)/6)
        rec_k=0
        self.rectangles = []
        self.hexagons = []        
        for ii in range(recObsNum):
            faa=recObs[rec_k:rec_k+4]
            polygon = shapely.geometry.Polygon(faa)
            self.workspace=shapely.difference(self.workspace,shapely.geometry.polygon.orient(shapelyObstacle.spawnPoly(faa),1.0))
            self.rectangles.append(polygon)
            rec_k += 4
        hex_k=0
        for ii in range(hexObsNum):
            faa = hexObs[hex_k:hex_k+6]
            polygon = shapely.geometry.Polygon(faa)
            self.workspace = shapely.ops.unary_union([self.workspace, polygon]).difference(polygon)
            self.hexagons.append(polygon)
            hex_k += 6

    def plotObstacles(self, viz):
        for rect in self.rectangles:
            x, y = rect.exterior.xy
            viz.plot(x, y, color='black')
            viz.fill(x, y, color='red', alpha=0.99)

        for hex in self.hexagons:
            x, y = hex.exterior.xy
            viz.plot(x, y, color='black')
            viz.fill(x, y, color='red', alpha=0.99)
    def polyNav(self, state, target):
            """
            Computes a navigation vector using artificial potential fields.
            
            Args:
                state: Current position [x, y]
                target: Target position [x, y]
                
            Returns:
                vec: Navigation vector [dx, dy] with shape (1,2)
            """
            # Parameters
            k_att = 1.0  # Attractive potential gain
            k_rep = 10.0  # Repulsive potential gain
            rho_0 = 5.0   # Influence distance of obstacles
            min_dist = 0.3  # Minimum distance to consider (to avoid singularities)
            max_attr_dist = 5.0  # Maximum distance for full attraction
            
            # Ensure state and target are 1D numpy arrays
            state = np.array(state).flatten()
            target = np.array(target).flatten()
            
            # Check dimensions
            if state.size != 2 or target.size != 2:
                print(f"Warning: Input dimensions unexpected. state shape: {state.shape}, target shape: {target.shape}")
                # Ensure we have exactly 2 elements (x,y)
                state = state[:2]
                target = target[:2]
            
            # Attractive potential gradient (towards target)
            dist_to_target = np.linalg.norm(target - state)
            
            if dist_to_target > max_attr_dist:
                # If far from target, constant attraction
                attr_vec = k_att * (target - state) / dist_to_target
            else:
                # If close to target, linear attraction
                attr_vec = k_att * (target - state)
            
            # Repulsive potential gradient (away from obstacles)
            rep_vec = np.zeros(2)
            
            # Process rectangles
            for rect in self.rectangles:
                # Find closest point on the rectangle
                closest_point = np.array(shapely.ops.nearest_points(
                    shapely.geometry.Point(state), rect)[1].coords[0])
                
                # Calculate distance to obstacle
                dist_to_obs = max(min_dist, np.linalg.norm(state - closest_point))
                
                if dist_to_obs < rho_0:
                    # Only apply repulsion within the influence distance
                    factor = k_rep * (1/dist_to_obs - 1/rho_0) * (1/dist_to_obs**2)
                    direction = (state - closest_point) / dist_to_obs
                    rep_vec += factor * direction
            
            # Process hexagons
            for hex in self.hexagons:
                # Find closest point on the hexagon
                closest_point = np.array(shapely.ops.nearest_points(
                    shapely.geometry.Point(state), hex)[1].coords[0])
                
                # Calculate distance to obstacle
                dist_to_obs = max(min_dist, np.linalg.norm(state - closest_point))
                
                if dist_to_obs < rho_0:
                    # Only apply repulsion within the influence distance
                    factor = k_rep * (1/dist_to_obs - 1/rho_0) * (1/dist_to_obs**2)
                    direction = (state - closest_point) / dist_to_obs
                    rep_vec += factor * direction
            
            # Combine attractive and repulsive vectors
            nav_vec = attr_vec + rep_vec
            
            # Normalize if vector is too large
            vec_magnitude = np.linalg.norm(nav_vec)
            max_speed = 1.0
            if vec_magnitude > max_speed:
                nav_vec = (nav_vec / vec_magnitude) * max_speed
            
            # Make sure to return a (1,2) shape array as expected
            return nav_vec.reshape(1, 2)

    def ispolycw(self, xy):
        return (self.polysignarea(xy) <= 0)

    def delta_gradient(self, q):
        """
        Compute the gradient (derivative) of delta with respect to q.
        
        Parameters:
        q : array-like
            The position to evaluate (2D point)
            
        Returns:
        numpy.ndarray : The gradient vector of delta at point q
        """        
        # Convert q to a Point object
        if not isinstance(q, shapely.geometry.Point):
            q_point = shapely.geometry.Point(q)
        else:
            q_point = q
        
        q_array = np.array([q_point.x, q_point.y])
        
        # Check if the point is outside the workspace
        if not self.workspace.contains(q_point):
            return np.zeros(2)  # Return zero gradient if outside workspace
        
        # Initialize with large distance
        min_distance = float('inf')
        nearest_point = None
        
        # Check workspace boundary
        workspace_boundary = self.workspace.boundary
        nearest_workspace_point = shapely.ops.nearest_points(q_point, workspace_boundary)[1]
        workspace_distance = q_point.distance(workspace_boundary)
        
        if workspace_distance < min_distance:
            min_distance = workspace_distance
            nearest_point = nearest_workspace_point
        
        # Check distance to each rectangle
        for rect in self.rectangles:
            rect_boundary = rect.boundary
            rect_nearest_point = shapely.ops.nearest_points(q_point, rect_boundary)[1]
            rect_distance = q_point.distance(rect_boundary)
            
            if rect_distance < min_distance:
                min_distance = rect_distance
                nearest_point = rect_nearest_point
        
        # Check distance to each hexagon
        for hex_poly in self.hexagons:
            hex_boundary = hex_poly.boundary
            hex_nearest_point = shapely.ops.nearest_points(q_point, hex_boundary)[1]
            hex_distance = q_point.distance(hex_boundary)
            
            if hex_distance < min_distance:
                min_distance = hex_distance
                nearest_point = hex_nearest_point
        
        # Compute gradient - unit vector pointing from q to the nearest point
        if nearest_point is not None:
            nearest_point_array = np.array([nearest_point.x, nearest_point.y])
            direction = nearest_point_array - q_array
            
            # Normalize to get unit vector
            norm = np.linalg.norm(direction)
            if norm > 0:
                gradient = direction / norm
            else:
                gradient = np.zeros(2)
        else:
            gradient = np.zeros(2)
        
        return gradient
    
    def polydist(self, xy, p):
        xy = xy.reshape(-1, 2)
        p = p.reshape(-1, 2)
        
        if xy.shape[0] == 0:
            D = np.zeros(p.shape[0])
            D.fill(np.inf)
            C = np.zeros(p.shape)
            C.fill(np.inf)
            return D, C
        
        orientsign = 1 - 2 * self.ispolycw(xy)
        numPoint = p.shape[0]
        
        xyPre = np.roll(xy, 1, axis=0)
        dxy = xyPre - xy
        dxyNorm = np.power(np.linalg.norm(dxy, axis=1)[:, np.newaxis], 2)
        dxyNorm[(dxyNorm == 0)] = 1
        
        D = np.zeros(numPoint)
        C = np.zeros([numPoint, 2])
        for k in range(numPoint):
            w = np.sum((p[k] - xy) * dxy, axis=1) / dxyNorm[:, 0]
            w = np.fmax(np.fmin(w, 1), 0)
            ctemp = (1 - w[:, np.newaxis]) * xy + w[:, np.newaxis] * xyPre
            dtemp = np.linalg.norm(p[k] - ctemp, axis=1)
            iMin = dtemp.argmin()
            D[k] = dtemp[iMin]
            C[k] = ctemp[iMin]
        
        return D, C

    def attractive_potential(self, state, target, k_att=1.0):
        return k_att * (np.array(target) - np.array(state))

    def repulsive_potential(self, state, k_rep=100.0, repulsion_radius=5.0):
        rep_force = np.zeros(2)
        for ob in self.rectangles + self.hexagons:
            ob_points = np.array(ob.exterior.coords)
            D, C = self.polydist(ob_points, np.array([state]))
            distance = D[0]
            nearest_point = C[0]
            if distance < repulsion_radius:
                direction = (np.array(state) - np.array(nearest_point)).flatten()
                rep_force += k_rep * (1.0 / distance - 1.0 / repulsion_radius) * (1.0 / distance**3) * direction
        return rep_force

    def compute_navigation_vector(self, state, target):
        attr_force = self.attractive_potential(state, target)
        rep_force = self.repulsive_potential(state)
        total_force = attr_force + rep_force
        norm = np.linalg.norm(total_force)
        if norm > 0:
            return total_force / norm  # Normalize to get a unit vector
        else:
            return total_force