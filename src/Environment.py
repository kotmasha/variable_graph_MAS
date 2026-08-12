
from numpy import linalg as la
import numpy as np
import random
import shapely
import shapely.plotting
import shapely.geometry as geom
from shapely.ops import unary_union
import reactive_planner_lib
from myGeometryTools import skewJ

from Obstacle import shapelyObstacle
# from scipy.optimize import minimize
from scipy.linalg import lu_solve, lu_factor, lu
from scipy.spatial import HalfspaceIntersection
#from scipy.spatial import Voronoi, voronoi_plot_2d

import qpsolvers
import math
# import cvxopt
import quadprog
import sys
class environment():
    def __init__(self,envInfo):
        # envInfo is the environment data read from the yaml file
        self.wkspcLowerBds=np.array([[(envInfo['WorkspaceBdry']['Xmin']),(envInfo['WorkspaceBdry']['Ymin'])]]).T # DWR 6/15/26 fixed array not transposing properly; added () around the envInfo calls
        self.wkspcUpperBds=np.array([[(envInfo['WorkspaceBdry']['Xmax']),(envInfo['WorkspaceBdry']['Ymax'])]]).T
        self.workspace=shapely.geometry.polygon.orient(shapely.Polygon([ 
            [envInfo['WorkspaceBdry']['Xmin'],envInfo['WorkspaceBdry']['Ymin']],
            [envInfo['WorkspaceBdry']['Xmax'],envInfo['WorkspaceBdry']['Ymin']],
            [envInfo['WorkspaceBdry']['Xmax'],envInfo['WorkspaceBdry']['Ymax']],
            [envInfo['WorkspaceBdry']['Xmin'],envInfo['WorkspaceBdry']['Ymax']],
            [envInfo['WorkspaceBdry']['Xmin'],envInfo['WorkspaceBdry']['Ymin']]
        ]),1.0) # shapely polygon whose interior is the desired workspace
        #DWR 6/15/26: the list of points in the four lines above needed to be converted to a shapely.polygon first
        self.obstacleData=envInfo['Obstacles']
        self.stateDim=envInfo['Dim']
        
    def nav(self,goal,state): #  6/8/2026
        return goal-state # default stand-in for a navigation field, to be re-defined in each subclass, like for sphere or polygon case
    
    def generateRndPoint(self, radius=None, refPt=None):  
        hitWorkspace=False
        while not hitWorkspace:
            if (radius is None) or (refPt is None):
                pt=np.array([[random.uniform(self.wkspcLowerBds(0),self.wkspcUpperBds(0))],[random.uniform(self.wkspcLowerBds(1),self.wkspcUpperBds(1))]])
                hitWorkspace=self.hitWkspc(pt)   
            else:   
                th=random.uniform(0,2*np.pi)
                rad=random.uniform(0,radius)
                pt=refPt+rad*np.array([[np.cos(th)],[np.sin(th)]])
                hitWorkspace=self.hitWkspc(pt)

    def hitWkspc(self,pt):
        return shapely.contains_xy(self.workspace,pt[0,0],pt[1,0])
    
    def ObsCheck(self,pos):
        return self.workspace.contains(shapely.geometry.Point(pos))
    
    def workspacePatch(self):
        return shapely.plotting.plot_polygon(self.workspace,add_points=False)


class sphereworldEnv(environment):
    def __init__(self,envInfo,visualize=False): # sets up polygonal workspace, workspace - obstacles
        #only ONE call should have visualize=True, the call in main_single_sim.py
        super().__init__(envInfo)

        self.obstacleNum=len(self.obstacleData)
        self.obstacleCenters=[] # Prep list of obstacle centers
        self.obstacleRadii=[]
        # For visualisation:
        ObstacleListTemp={}
        for obs in self.obstacleData:
            ObstacleListTemp[obs]=shapely.geometry.Point(self.obstacleData[obs]['center']).buffer(self.obstacleData[obs]['radius'])
        self.ObstacleList=shapely.geometry.MultiPolygon([ObstacleListTemp[obs] for obs in ObstacleListTemp])

        # Loop over all the obstacles, fill the lists, and update the workspace (removing obstacles one by one)
        for ObsName in self.obstacleData:
            center=self.obstacleData[ObsName]['center']
            self.obstacleCenters.append(center)
            radius=self.obstacleData[ObsName]['radius']
            self.obstacleRadii.append(radius)
            self.workspace=shapely.difference(self.workspace,shapely.geometry.polygon.orient(shapelyObstacle.spawnSphere(center,radius),1.0))
        # Transform lists into numpy arrays
        self.obstacleCenters=np.matrix(self.obstacleCenters)
        self.obstacleRadii=np.matrix(self.obstacleRadii).T

    def nav(self,goal,pos): # both goal and state are assumed to be numpy column vector matrices
        # set up a qp-solve problem for the projection of the goal to the safe polygon
        goal=np.array(goal) # DWR 6/23/2026 followup: Unfortunately, qpsolvers does not like matrices.
        pos=np.array(pos)
        prob=qpsolvers.problem.Problem(
            np.eye(np.size(goal)), # minimizing squared norm
            np.zeros((np.size(goal),1)), # no linear component in this QP
            G=self.safetyMatrixExtended(pos), # safety constraints matrix (extended version, with workspace boundary)
            h=self.safetyCoefficientsExtended(goal,pos), # safety constraints coefficients (extended version, with workspace boundary)
            #lb=0.5*(self.wkspcLowerBds+pos)-goal, # workspace boundary-safety lower bounds
            #ub=0.5*(self.wkspcUpperBds+pos)-goal, # workspace boundary-safety upper bounds
            #initvals=(pos-goal) # interior point of the polygon for the convenience of the qp solver
        )
        #Note: cannot create initvals in the Problem former itself
        #solve the QP problem
        #solver list: https://pypi.org/project/qpsolvers/
        result=qpsolvers.solve_qp(P=prob.P,q=prob.q,G=prob.G,h=prob.h,solver='piqp',initvals=(pos-goal))
        if result is None:
            result=np.zeros((np.size(goal),1))
        return np.matrix(goal+result.reshape((np.size(goal),1))-pos)
    
    def safetyMatrix(self,pos):
        # Computes the coefficient matrix describing the safe polytope at the point z
        return np.subtract(self.obstacleCenters,pos.T)

    def safetyMatrixExtended(self,pos):
        # Computes the coefficient matrix describing the safe polytope at the point z
        # For use in the safe polygon visualization
        M=np.subtract(self.obstacleCenters,pos.T)
        return np.vstack((M,np.matrix([[1,0],[0,1],[-1,0],[0,-1]]))) # append workspace boundary equations (upper bounds, then lower bounds)

    def obstacleDist(self,pos):
        # Computes the column vector of distances of z to the obstacle centers
        return la.norm(np.subtract(self.obstacleCenters,pos.T),axis=1,keepdims=True)
    
    def obstacleDistExtended(self,pos):
        # Computes the column vector of distances of z to the obstacle centers
        ballDists=la.norm(np.subtract(self.obstacleCenters,pos.T),axis=1,keepdims=True)
        wkspcDists=np.abs(np.array([
            [pos[0][0]-self.wkspcLowerBds[0][0]],
            [pos[1][0]-self.wkspcLowerBds[1][0]],
            [pos[0][0]-self.wkspcUpperBds[0][0]],
            [pos[1][0]-self.wkspcUpperBds[1][0]]
            ]))
        return np.vstack((ballDists,wkspcDists))

    def safetyCoefficientsExtended(self,goal,pos): # pos is a 2-by-1 vector, for visualizations
        # For use in the safe polygon visualization
        # input should be column vectors
        # two ways to do this: only consider
        dists=self.obstacleDist(pos)
        cons=self.safetyMatrix(pos)
        b=np.array(0.5*(np.power(dists,2) - np.multiply(self.obstacleRadii.reshape(-1,1),dists))+cons @ (pos-goal.reshape((np.size(goal),1))))
        b=np.vstack((b,(0.5*(self.wkspcUpperBds+pos)-goal),-(0.5*(self.wkspcLowerBds+pos)-goal))) # append workspace boundary upper bounds, then lower bounds
        return b

    def safetyCoefficientsNoGoal(self,pos): # pos is a 2-by-1 vector, for visualizations
            # For use in the safe polygon visualization
            # input should be column vectors
            # two ways to do this: only consider
            dists=self.obstacleDist(pos)
            cons=self.safetyMatrix(pos)
            b=np.array(0.5*(np.power(dists,2) - np.multiply(self.obstacleRadii.reshape(-1,1),dists))+cons @ pos)
            b=np.vstack((b,0.5*(self.wkspcUpperBds+pos),-0.5*(self.wkspcLowerBds+pos))) # append workspace boundary upper bounds, then lower bounds
            return b
    
    def safetyCoefficients(self,goal,pos): # goal and pos are 2-by-1 vectors, for use with self.nav
        # may want to delete this, superseded by Extended version

        dists=self.obstacleDist(pos)
        cons=self.safetyMatrix(pos)
        b=np.array(0.5*(np.power(dists,2) - np.multiply(self.obstacleRadii.reshape(-1,1),dists))+cons @ (pos-goal.reshape((np.size(goal),1))))
        return b

    def inCircle(self,idx,idy,oc,oR):
        return (idx-oc[0])**2 + (idy-oc[1])**2 <= oR**2
    
    def isSafe(self,idx,idy):
        count=0
        for i in range(self.obstacleNum): 
            oc=self.obstacleCenters[i]
            oR=self.obstacleRadii[i]
            if self.inCircle(idx,idy,oc,oR):
                count=count + 1 # Point is inside an obstacle, not safe
        if count > 0:
            return False
        else:
            return True


    # def plotObstacles(self,viz): # DWR 7/13/2026: Code currently uses workspace-obstacles directly for plotting
    #     for i in range(self.obstacleNum):
    #         oc=shapely.geometry.Point(self.obstacleCenters[i])
    #         oR=(self.obstacleRadii[i]).item()
    #         circ=oc.buffer(oR)
    #         x,y=circ.exterior.xy
    #         viz.plot(x, y, color='black')
    #         viz.fill(x, y, color='gray',alpha=1)



class polygonEnv(environment):    
    def __init__(self,envInfo):
        super().__init__(envInfo)
        #self.DiffeoParams={np.float64(envInfo['DiffeoParams'][key]) for key in envInfo['DiffeoParams']}
        self.DiffeoParams={}
        for key in envInfo['DiffeoParams']:
            self.DiffeoParams[key]=np.float64(envInfo['DiffeoParams'][key])            
        Xmin=envInfo['WorkspaceBdry']['Xmin']
        Xmax=envInfo['WorkspaceBdry']['Xmax']
        Ymin=envInfo['WorkspaceBdry']['Ymin']
        Ymax=envInfo['WorkspaceBdry']['Ymax']
        self.workspaceVertices=[[Xmin,Ymin],[Xmax,Ymin],[Xmax,Ymax],[Xmin,Ymax],[Xmin,Ymin]]
        #For use in visualisation:
        self.workspace=shapely.geometry.Polygon(self.workspaceVertices)
        for obs in self.obstacleData:
            obstacle_poly=shapely.geometry.Polygon(self.obstacleData[obs]['vertices'])
            self.workspace=shapely.difference(self.workspace, shapely.geometry.polygon.orient(obstacle_poly, 1.0))
        self.ObstacleList=shapely.geometry.MultiPolygon([shapely.geometry.Polygon(self.obstacleData[obs]['vertices']) for obs in self.obstacleData])
        shapely.prepare(self.ObstacleList)

        # k=0
        # self.obstacle_polygons=[]  # Store obstacle polygons for distance calculations
        # for ii in range(obsNum):
        #     faa=obs[k:k+4]
        #     obstacle_poly=shapelyObstacle.spawnPoly(faa)
        #     self.obstacle_polygons.append(np.array(obstacle_poly.exterior.coords))
        #     self.workspace=shapely.difference(self.workspace, shapely.geometry.polygon.orient(obstacle_poly, 1.0))
        #     k += 4

        # DWR 7/8/2026: Plan: 
        #   1: Maybe rewrite/replace above code
        #   2: Create the polygon triangulations/trees IN INIT. We don't need to recalculate the trees every time we do the diffeomorphism
        #   3: The obstacles in the yml should be arrays of points made by the users.
        
        # DWR 7/13/2026: create the visualization here instead of network.py

        self.sphereWorldParams=envInfo
        self.obstacleTrees={}
        for obs in self.obstacleData:
            #Vertices should be nparray Nx2
            vertices=np.array(self.obstacleData[obs]['vertices'])
            self.obstacleTrees[obs]=reactive_planner_lib.diffeoTreeTriangulation(vertices,self.DiffeoParams,self.workspaceVertices)
            self.sphereWorldParams['Obstacles'][obs]['center']=self.obstacleTrees[obs][-1]['center'][0] # added [0] to un-nest the array
            self.sphereWorldParams['Obstacles'][obs]['radius']=self.obstacleTrees[obs][-1]['radius']
            # DWR 7/11/2026: Wasn't sure whether to "un-nest" the array here or in diffeoTree, since it may have been intentional design to mimic matrices

        # Construct the corresponding sphere world environment
        # 1. construct sphereWorldParams
        # 2. call the sphereWorldEnv constructor
        self.sphereWorld=sphereworldEnv(self.sphereWorldParams)

    # DWR 7/8/2026: Plan: 
    #   1: Polygon->Sphere diffeo
    #   2: Sphereworld nav (already done)
    #   3: Reverse diffeo
    #   4: return vector
    def nav(self,goal,pos):
        goal=np.array(goal).reshape(1,-1)
        pos=np.array(pos).reshape(1,-1)

        diffeoPos=pos
        diffeoPosD=np.eye(2)
        diffeoPosDD=np.zeros([1,8])[0]
        for obs in self.obstacleTrees:
            #DiffeoParams should be dictionary. The question is where do we create DiffeoParams? Probably the yml.
            spherePos,spherePosD,spherePosDD=reactive_planner_lib.polygonDiffeoTriangulation(diffeoPos,self.obstacleTrees[obs],self.DiffeoParams)

            res0=spherePosD[0,0]*diffeoPosDD[0] + spherePosD[0,1]*diffeoPosDD[4] + diffeoPosD[0,0]*(spherePosDD[0]*diffeoPosD[0,0] + spherePosDD[1]*diffeoPosD[1,0]) + diffeoPosD[1,0]*(spherePosDD[2]*diffeoPosD[0,0] + spherePosDD[3]*diffeoPosD[1,0])
            res1=spherePosD[0,0]*diffeoPosDD[1] + spherePosD[0,1]*diffeoPosDD[5] + diffeoPosD[0,0]*(spherePosDD[0]*diffeoPosD[0,1] + spherePosDD[1]*diffeoPosD[1,1]) + diffeoPosD[1,0]*(spherePosDD[2]*diffeoPosD[0,1] + spherePosDD[3]*diffeoPosD[1,1])
            res2=spherePosD[0,0]*diffeoPosDD[2] + spherePosD[0,1]*diffeoPosDD[6] + diffeoPosD[0,1]*(spherePosDD[0]*diffeoPosD[0,0] + spherePosDD[1]*diffeoPosD[1,0]) + diffeoPosD[1,1]*(spherePosDD[2]*diffeoPosD[0,0] + spherePosDD[3]*diffeoPosD[1,0])
            res3=spherePosD[0,0]*diffeoPosDD[3] + spherePosD[0,1]*diffeoPosDD[7] + diffeoPosD[0,1]*(spherePosDD[0]*diffeoPosD[0,1] + spherePosDD[1]*diffeoPosD[1,1]) + diffeoPosD[1,1]*(spherePosDD[2]*diffeoPosD[0,1] + spherePosDD[3]*diffeoPosD[1,1])
            res4=spherePosD[1,0]*diffeoPosDD[0] + spherePosD[1,1]*diffeoPosDD[4] + diffeoPosD[0,0]*(spherePosDD[4]*diffeoPosD[0,0] + spherePosDD[5]*diffeoPosD[1,0]) + diffeoPosD[1,0]*(spherePosDD[6]*diffeoPosD[0,0] + spherePosDD[7]*diffeoPosD[1,0])
            res5=spherePosD[1,0]*diffeoPosDD[1] + spherePosD[1,1]*diffeoPosDD[5] + diffeoPosD[0,0]*(spherePosDD[4]*diffeoPosD[0,1] + spherePosDD[5]*diffeoPosD[1,1]) + diffeoPosD[1,0]*(spherePosDD[6]*diffeoPosD[0,1] + spherePosDD[7]*diffeoPosD[1,1])
            res6=spherePosD[1,0]*diffeoPosDD[2] + spherePosD[1,1]*diffeoPosDD[6] + diffeoPosD[0,1]*(spherePosDD[4]*diffeoPosD[0,0] + spherePosDD[5]*diffeoPosD[1,0]) + diffeoPosD[1,1]*(spherePosDD[6]*diffeoPosD[0,0] + spherePosDD[7]*diffeoPosD[1,0])
            res7=spherePosD[1,0]*diffeoPosDD[3] + spherePosD[1,1]*diffeoPosDD[7] + diffeoPosD[0,1]*(spherePosDD[4]*diffeoPosD[0,1] + spherePosDD[5]*diffeoPosD[1,1]) + diffeoPosD[1,1]*(spherePosDD[6]*diffeoPosD[0,1] + spherePosDD[7]*diffeoPosD[1,1])
            diffeoPosDD[0]=res0
            diffeoPosDD[1]=res1
            diffeoPosDD[2]=res2
            diffeoPosDD[3]=res3
            diffeoPosDD[4]=res4
            diffeoPosDD[5]=res5
            diffeoPosDD[6]=res6
            diffeoPosDD[7]=res7

            diffeoPosD=spherePosD*diffeoPosD
            diffeoPos=spherePos

            # compute the goal in sphere world:
            sphereGoal,_,_=reactive_planner_lib.polygonDiffeoTriangulation(goal,self.obstacleTrees[obs],self.DiffeoParams)

        # compute the navigation field value in sphere world:
        sphereNavField=self.sphereWorld.nav(sphereGoal.transpose(),spherePos.transpose())

        # compute and return the pull-back of sphereNavField to the real world:
        #   DWR 7/13/2026: we replaced np.matmul with lu_solve for better numerical stability
        #return np.matmul(np.linalg.inv(diffeoPosD),sphereNavField)
        return lu_solve(lu_factor(diffeoPosD),sphereNavField,overwrite_b=True) # Performance looks slightly better, but haven't tested it much
    
    def obstacleBufferPlot(self): # used in main_single_sim to plot stuff
        bufferedPolygons=shapely.buffer(self.ObstacleList,self.DiffeoParams['epsilon'])
        bufferedPolygons=shapely.difference(bufferedPolygons,self.ObstacleList) # Removes obstacle from buffer for visual clarity
        return shapely.plotting.plot_polygon(bufferedPolygons,add_points=False,color='red',)
    
    def collarPolygonPlot(self): # used in main_single_sim to plot stuff
        collarList=[]
        for obs in self.obstacleTrees:
            for i in range(0,len(self.obstacleTrees[obs])):
                collarList.append(shapely.geometry.Polygon(self.obstacleTrees[obs][i]['vertices_tilde']))
        multiPoly=shapely.geometry.MultiPolygon(collarList)
        return shapely.plotting.plot_polygon(multiPoly,add_points=False,color='blue',)
    
    # def plotObstacles(self,viz): # DWR 7/13/2026: Code currently uses workspace minus obstacles directly for plotting
    #     for obs in self.obstacleData:
    #         poly=shapely.geometry.Polygon(self.obstacleData[obs]['vertices'])
    #         poly=shapely.geometry.polygon.orient(poly,1.0)
    #         x, y=poly.exterior.xy
    #         viz.plot(x, y, color='black')
    #         viz.fill(x, y, color='gray',alpha=0.3)
    
    def polydist(self, xy, p): # DWR 7/8/2026: Is there a reason we can't replace this with polydist from polygeom_lib?
        """
        Computes the distance between a set of points, p, and 
        a polygon, xy, and returns the closest points on the polygon boundary.
        
        Input:  
            xy : Vertex coordinates of a polygon (Nx2 numpy.array)
            p  : Coordinates of a set of points (Mx2 numpy.array)
        Output: 
            D  : Distance between points and the polygon 
            C  : Coordinates of the closest points on the polygon to the input points
        """
        # Convert input data into 2D arrays
        xy=xy.reshape(-1, 2)
        p=p.reshape(-1, 2)
         
        # Distance to empty set is infinity
        if (xy.shape[0] == 0):
            D=np.zeros(p.shape[0])
            D.fill(np.inf)
            C=np.zeros(p.shape)
            C.fill(np.inf) 
            return D, C
        
        orientsign=1 - 2 * self.ispolycw(xy)  # orientation of the polygon
        numPoint=p.shape[0]  # number of points
        # Relative coordinates of polygon rims
        xyPre=np.roll(xy, 1, axis=0)
        dxy=xyPre - xy
        dxyNorm=np.power(np.linalg.norm(dxy, axis=1)[:, np.newaxis], 2)
        dxyNorm[(dxyNorm == 0)]=1

        # Compute distances and closest points on the polygon boundary  
        D=np.zeros(numPoint)
        C=np.zeros([numPoint, 2])
        for k in range(numPoint):
            w=np.sum((p[k] - xy) * dxy, axis=1)[:, np.newaxis] / dxyNorm
            w=np.fmax(np.fmin(w, 1), 0)
            ctemp=(1 - w) * xy + w * xyPre
            dtemp=np.linalg.norm(p[k] - ctemp, axis=1)
            iMin=dtemp.argmin()
            D[k]=dtemp[iMin]
            C[k]=ctemp[iMin]  
        
        return D, C
    
    def ispolycw(self, xy): # DWR 7/8/2026: Is only used in this version of polydist, which we are not sure we need.
        """
        Determines if the vertices, xy, of a non-self-intersecting polygon 
        are in clockwise order based on the signed area of the polygon.
        
        Input:
            xy : Vertex coordinates of a non-self-intersecting polygon (Nx2 numpy.array)   
        Output:
            cw : Boolean True if the input polygon is in clockwise order
        """
        return (self.polysignarea(xy) <= 0)
    
    def polysignarea(self, xy): # DWR 7/8/2026: Can probably be replaced with polysignedarea in polygeom_lib
        """
        Determines the signed area of a non-self-intersecting polygon with vertices xy
        
        Input:
            xy   : Vertex coordinates of a non-self-intersecting polygon (Nx2 numpy.array)   
        Output:
            area : Signed area of the polygon
        """
        xy=xy.reshape(-1, 2)  # Convert the input data into a 2D array 
        numVertex=xy.shape[0]  # Number of vertices
        area=0.0
        for ck in range(0, numVertex):
            cn=(ck + 1) % numVertex
            area=area + np.cross(xy[ck], xy[cn])
        area=0.5 * area

        return area
    
    def get_distance_to_nearest_obstacle(self, state): # Can probably be removed too
        """
        Calculates the minimum distance from a given state to any obstacle in the environment.
        
        Input:
            state : Position coordinates [x, y] as numpy array or list
        Output:
            min_dist : Minimum distance to the nearest obstacle
            nearest_point : Coordinates of the nearest point on the obstacle
        """
        state=np.array(state).reshape(1, 2)  # Ensure state is in correct shape
        
        min_dist=float('inf')
        nearest_point=None
        
        for obstacle_poly in self.obstacle_polygons:
            # Calculate distance to this obstacle
            distances, closest_points=self.polydist(obstacle_poly, state)
            
            # Check if this is the closest obstacle so far
            if distances[0] < min_dist:
                min_dist=distances[0]
                nearest_point=closest_points[0]
        
        return min_dist, nearest_point

    def unicycleNavTestVis(self,goal,pos):
        return 1
        
    # \nRadius of\ncommunication=3m



# class starworldEnv(environment):
#     def __init__(self,outerbounds,obstacleData):
#         super().__init__(outerbounds,obstacleData)
        
#         self.obstacleNumPoints=360
#         self.obstacleNum=self.obstacleData['starWorld']['obsNum']
#         self.obstacleCenters=np.ones((self.obstacleNum,self.stateDim))
#         self.obstacleRadii=np.ones((self.obstacleNum))
        
#         for ii in range(self.obstacleNum):
#             theta=2*np.pi*(ii-1)/self.obstacleNum
#             self.obstacleCenters[ii,:]=np.array([8*np.cos(theta),8*np.sin(theta)])
#             self.obstacleRadii[ii]=ii+0.5
#             if ii ==2:
#                 self.obstacleRadii[ii]=ii-0.5
#         # print(self.obstacleCenters,self.obstacleRadii)
#         self.obstacleClearance=8*np.array((self.obstacleData['starWorld']['obsClearance']))
#         print(self.obstacleClearance)
#         angles=np.linspace(0, 2 * np.pi, self.obstacleNumPoints, endpoint=False)
#         angles=np.append(angles, 0) 
#         obstacleBufferList=[]

#         for ii in range(self.obstacleNum):
#             oc=self.obstacleCenters[ii, :]
#             radii=self.barrierCurve(ii, angles)
            
#             # Create star-shaped obstacle polygon
#             x, y=radii * np.cos(angles), radii * np.sin(angles)
#             x += oc[0]
#             y += oc[1]
#             obstacle=shapely.Polygon(zip(x, y)).buffer(0)
            
#             # Subtract obstacle from workspace
#             self.workspace=shapely.difference(self.workspace, obstacle)
            
#             # Create buffered region around the obstacle
#             radii_buffer=np.sqrt(self.barrierCurve(ii, angles)**2 + self.obstacleClearance[ii]**2)
#             x_buffer, y_buffer=radii_buffer * np.cos(angles), radii_buffer * np.sin(angles)
#             x_buffer += oc[0]
#             y_buffer += oc[1]
            
#             # Create obstacle buffer polygon
#             obstacle_buffer=shapely.Polygon(zip(x_buffer, y_buffer)).buffer(0)  # Buffer set to 0 to create a polygon only
#             # self.workspace=shapely.difference(self.workspace, obstacle_buffer)
#             obstacleBufferList.append(obstacle_buffer)
#             # Intersect with workspace
#             # obstacle_buffer_set=obstacle_buffer_set
            
#             # Add to the union of buffers
#             # obstacleBuffer=unary_union([obstacleBuffer, obstacle_buffer])


#     def plotObstacles(self,viz):
#         angles=np.linspace(0, 2 * np.pi, self.obstacleNumPoints, endpoint=False)
#         angles=np.append(angles, 0) 
#         obstacleBufferList=[]

#         for ii in range(self.obstacleNum):
#             oc=self.obstacleCenters[ii, :]


#             # Create buffered region around the obstacle
#             radii_buffer=np.sqrt(self.barrierCurve(ii, angles)**2 + self.obstacleClearance[ii]**2)
#             x_buffer, y_buffer=radii_buffer * np.cos(angles), radii_buffer * np.sin(angles)
#             x_buffer += oc[0]
#             y_buffer += oc[1]
            
#             # Create obstacle buffer polygon
#             obstacle_buffer=shapely.Polygon(zip(x_buffer, y_buffer)).buffer(0)  # Buffer set to 0 to create a polygon only
#             # Plot the obstacle buffer with alpha=0.7
#             viz.fill(x_buffer, y_buffer, alpha=0.7, facecolor='green')
#             radii=self.barrierCurve(ii, angles)
            
#             # Create star-shaped obstacle polygon
#             x, y=radii * np.cos(angles), radii * np.sin(angles)
#             x += oc[0]
#             y += oc[1]
#             obstacle=shapely.Polygon(zip(x, y)).buffer(0)
            
#             # Subtract obstacle from workspace
#             self.workspace=shapely.difference(self.workspace, obstacle)
#             viz.fill(x, y, color='red',alpha=0.99)
#             # viz.plot(obstacle_buffer, ax=viz, alpha=0.7, facecolor='green')

        
        
#     def navfSphere(self,goal,state):
#         self.A=np.vstack((self.workspaceMatrix,self.safetyMatrix(state)))
#         self.bb=np.vstack((self.workspaceCoefficients-(self.workspaceMatrix@goal).reshape(len(self.workspaceMatrix),1),self.safetyCoefficients(goal,state)))
#         result=qpsolvers.solve_qp(np.eye(2),np.zeros((2,1)),self.A,self.bb,solver='piqp')
        
#         return goal+result.reshape((2,1))-state
    
#     def safetyMatrix(self,state):
#         # Computes the coefficient matrix describing the safe polytope at the point z

#         m=np.zeros((self.obstacleNum,self.stateDim))
#         for i in range(self.obstacleNum):
#             m[i,:]=self.obstacleCenters[i]-state.T

#             # pdb.set_trace()
#         return m

#     def obstacleDist(self,state):
#         # Computes the column vector of distances of z to the obstacle centers
#         c=np.zeros((self.obstacleNum,1))
#         for i in range(self.obstacleNum):
#             col=state - self.obstacleCenters[i,:].reshape((2,1))
#             c[i,:]=np.sqrt(col.T @ col)
#             # c[i]=la.norm(state-self.obstacleCenters[i].T)
#         return c     
    
#     def safetyCoefficients(self,goal,state):
#         # input should be column vectors
#         b=np.zeros((self.obstacleNum,1))
#         dists=self.obstacleDist(state)
#         cons=self.safetyMatrix(state)
#         b=b+0.5*(dists*dists-self.obstacleRadii.reshape(-1,1)*dists)+cons @ (state-goal.reshape((2,1))) 
#         return b
    
#     # def navfStar(self,goal,state):
#     #     # ensure state,goal are coloumn vecs
#     #     # print(state,goal)
#     #     newG=self.wksp2sph(goal)
#     #     newState=self.wksp2sph(state)
#     #     # A=np.linalg.det(self.wksp2sphDeriv(state))
#     #     return np.linalg.inv(self.wksp2sphDeriv(state))*self.navfSphere(newG,newState)
    
#     def navfStar(self, state, goal):
#         # Ensure state and goal are column vectors
#         state=state.reshape((2, 1))
#         goal=goal.reshape((2, 1))

#         # Transform state and goal to sphere coordinates
#         newG=self.wksp2sph(goal)
#         newState=self.wksp2sph(state)

#         # Calculate navigation function in sphere coordinates
#         nav_sphere=self.navfSphere(newG, vnewState)

#         # Calculate the Jacobian of the workspace-to-sphere transformation
#         J=self.wksp2sphDeriv(state)

#         # Transform the navigation vector back to workspace coordinates
#         nav_star=np.linalg.solve(J, nav_sphere)
#         print(f'nav_star: {nav_star}')
#         return nav_star

#     def wksp2sph(self,p):
#         stitches=self.stitchingMap(p)
#         q=stitches[self.obstacleNum,0]*p
#         # print(f'i am0:{q}')
#         for ii in range(self.obstacleNum):
#             oc=self.obstacleCenters[ii,:].reshape((2,1))
#             rad=self.obstacleRadii[ii]
#             stitch=stitches[ii,0]
#             # print(p,oc)
#             relpos=p-oc
#             # print(relpos)
#             relnorm=la.norm(relpos)
#             q=q+stitch*(oc+rad*relpos/relnorm)
#         # print(f'i am1:{q}')
#         return q
    
#     def wksp2sphDeriv(self,p):
#         stitches=self.stitchingMap(p)
#         print('in function wksp2deriv',stitches)
#         stitchesDeriv=self.smDeriv(p)
#         D=stitches[self.obstacleNum]*np.eye(self.stateDim)

#         for ii in range(self.obstacleNum):
#             oc=self.obstacleCenters[ii,:].T
#             rad=self.obstacleRadii[ii]
#             stitch=stitches[ii,0]
#             relpos=p-oc
#             relnorm=la.norm(relpos)
#             D=D+(stitch*rad/relnorm)*np.eye(self.stateDim)
#             D=D+((rad/relnorm)-1)*relpos*stitchesDeriv[ii,:]
#             D=D-stitch*rad*(relnorm**-3)*(relpos*relpos.T)
#         return D
    
#     def stich(self,i,x):
#         epsilon=self.obstacleClearance[i]
#         out=self.bump(epsilon-x)/self.bump(epsilon)
#         return out
#     def stitchDeriv(self,i,x):
#         epsilon=self.obstacleClearance[i]
#         print(epsilon,x,epsilon-x)
#         print('in stich deriv',self.bumpDeriv(epsilon-x),self.bump(epsilon))
#         return -self.bumpDeriv(epsilon-x)/self.bump(epsilon)
    
#     def stitchingMap(self,p):
#         v=np.zeros([self.obstacleNum+1,1])
#         v[self.obstacleNum,0]=1
#         # print(f'p:{p}')
#         for ii in range(self.obstacleNum):
#             v[ii,0]=v[ii,0]+self.stich(ii,self.barrier(ii,p))
#             # print(v,ii)
#             v[self.obstacleNum]=v[self.obstacleNum,0]-v[ii,0]

#         return v
    
#     def smDeriv(self,p):
#         out=np.zeros((self.obstacleNum+1,self.stateDim))
#         for ii in range(self.obstacleNum):
#             # print()
#             print('in function smDeriv',self.stitchDeriv(ii,self.barrier(ii,p)),self.barrierDeriv(ii,p))
#             if self.stitchDeriv(ii,self.barrier(ii,p)) == np.zeros((1,1)):
#                 out[ii,:]=np.zeros((1,2))
#             else:

#                 out[ii,:]=(self.stitchDeriv(ii,self.barrier(ii,p))) @ self.barrierDeriv(ii,p)
        
#         out[ii,:]=-np.sum(out)
#         return out

    
#     def barrier(self,i,p):

#         # 
#         obsC=self.obstacleCenters[i].reshape((2,1))
#         # print(f'check if vert: {obsC} ')
#         # print(f'p:{p}')
#         relpos=p - obsC
#         # print(relpos)
#         rho= np.sqrt(relpos[0]**2 + relpos[1]**2)
#         # print(f'rho:{rho}')
#         theta=np.arctan2(relpos[1], relpos[0])
#         r0=self.barrierCurve(i,theta)
#         # print(rho**2-r0**2)
#         return rho**2-r0**2
    
#     def barrierDeriv(self,i,p):

#         obsC=self.obstacleCenters[i].reshape((2,1))
#         # print(f'check if vert {obsC} ')
#         obsR=self.obstacleRadii[i]
#         relpos=p - obsC
#         rho= np.sqrt(relpos[0]**2 + relpos[1]**2)
#         theta=np.arctan2(relpos[1], relpos[0])
#         u=np.array([np.cos(theta),np.sin(theta)])
#         v=np.array([np.sin(theta),-np.cos(theta)])
#         r0=self.barrierCurve(i,theta)
#         r1=self.barrierCurveDeriv(i,theta)
#         # print(2*(relpos+(r0*r1/rho)*v))
#         return 2*(relpos+(r0*r1/rho)*v).reshape((1,2))
    
#     def barrierCurve(self,i,th):
#         obsR=self.obstacleRadii[i]
#         if i ==0:
#             nPeaks=3
#             out=1.1*obsR+(1+np.sin(nPeaks*th))
#             return out
#         elif i==1:
#             nPeaks=7
#             out=1.1*obsR+(1+np.sin(nPeaks*th))
#             return out
#         elif i==2:
#             offsetAngle=np.pi/6
#             nPeaks=2
#             out=1.1*obsR+2*(1+np.cos(nPeaks*(th+offsetAngle)))
#             return out
        
    
#     def barrierCurveDeriv(self,i,th):

#         obsR=self.obstacleRadii[i]
#         if i ==0:
#             nPeaks=3
#             out=nPeaks*np.cos(nPeaks*th)
#         elif i==1:
#             nPeaks=7
#             out=nPeaks*np.cos(nPeaks*th)
#         elif i==2:
#             # offsetAngle=np.pi/6
#             nPeaks=2
#             out=-2*nPeaks*np.sin(nPeaks*(th))

#         return out
#     def bump(self,x):
#         return self.cInftyBump(x)
    
#     def bumpDeriv(self,x):
#         # print(f'i am x:{x}')
#         return self.cInftyBumpDeriv(x)
    
#     def cInftyBump(self, x):
#         """C^\infty bump function."""
#         # print("in cinfbump",x)
#         if x <= 0:
#             return np.zeros((1,1))
#         else:
#             # print('waatt',np.exp(-1 / x))
#             return np.exp(-1 / x)
    
#     def cInftyBumpDeriv(self, x):
#         """Derivative of the C^\infty bump function."""
#         if x <= 0:
#             return np.zeros((1,1))
#         else:
#             return np.exp(-1 / x) * (x ** -2)
    
#     def cOneBump(self, x):
#         """C^1 bump function."""
#         if x <= 0:
#             return 0
#         else:
#             return x ** 2
    
#     def cOneBumpDeriv(self, x):
#         """Derivative of C^1 bump function."""
#         if x <= 0:
#             return 0
#         else:
#             return 2 * x
    
#     def cTwoBump(self, x):
#         """C^1 bump function."""
#         if x <= 0:
#             return 0
#         else:
#             return x ** 3
    
#     def cTwoBumpDeriv(self, x):
#         """Derivative of C^1 bump function."""
#         if x <= 0:
#             return 0
#         else:
#             return 3 * x ** 2
