from numpy import linalg as la
import numpy as np
import os,sys
import math
import agent
from agent import Agent
from agent import unicycleAgent
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import shapely
import universal as uv
from Navigation import navigation
from agentTask import agentask
import inspect
from itertools import combinations
from datetime import datetime
from shapely.geometry import MultiPolygon, Polygon

class netwk():

    def __init__(self,netID,graphWithNames,env,leaders,pnpParameters,agentSpawn,simTime,worldType,randomQ,stateWname=None):
        #init mainly works on figure and setting animation visual objects 
        self.graph=graphWithNames
        self.agentNum=len(self.graph.names)
        self.env=env
        self.agents={name:None for name in self.graph.names}
        self.netID=netID
        self.rcomm=pnpParameters['rcomm']
        self.rsafe=pnpParameters['rsafe']*pnpParameters['rcomm']
        self.alpha=pnpParameters['alpha']
        self.leaderGain=pnpParameters['leaderGain']
        self.LazyQ=bool(pnpParameters['Lazy'])
        self.coopGain=int(pnpParameters['coopGain'])
        self.stateWname=stateWname
        self.timestart=0.0
        self.agentSpawn=agentSpawn
        self.simTime=simTime
        self.randomQ=randomQ
        self.worldType = worldType
        self.dt=0.0001
        

        if self.agentSpawn==1:
            self.mode=1
        elif self.agentSpawn==2:#uni
            self.mode=2
        else:
            self.mode=0

        self.leaders=leaders
        self.m=1/pnpParameters['rsafe']


        if self.graph.edges!=None:

            self.numEdges=len(self.graph.edges)
        
            if self.LazyQ:
                self.rsafeStar=self.rsafe+(self.rcomm-self.rsafe)/(self.numEdges**(1/(2+self.alpha)))

                self.omega=self.coopGain        
            else:
                self.rsafeStar=self.rsafe
                self.omega=self.coopGain*((2+self.alpha)/(2*self.rcomm**(1+self.alpha)))*(self.m**(1+self.alpha)*(self.numEdges-self.m**2))/((self.m-1)**(2+self.alpha))
        # self.interiorPt=np.array([-5,-5])
        # if experiment requires agents
        
        #unicycle stuff here
        if agentSpawn==2:
            self.task=agentask(self.graph,self.leaders,uniAgent=1)
            if self.randomQ:
                self.mode=3
            else:
                self.mode==2
            self.populate(self.mode)
            self.y0=np.empty((0,1))
            # for name in self.graph.names:
            #     self.y0=np.vstack((self.y0,self.agents[name].pos))
            self.figure,self.visualization=plt.subplots()
            self.workspacePatch=shapely.plotting.plot_polygon(self.env.workspace,add_points=False)
            self.visualization.add_patch(self.workspacePatch)
            radius=0.9
            self.arrowLen=0.8 # make this the linear speed ############
            self.verticesVisual={}
            self.arrowVisual={}
            for name in self.graph.names:
                pos = uv.col2tup(self.agents[name].pos)
                pose=uv.col2tup(self.agents[name].pose)
                circle = patches.Circle(pos, radius, color='orange', alpha=0.4)
                self.visualization.add_patch(circle)
                self.verticesVisual[name]=circle
                arrow_dx=pose[0]*self.arrowLen
                arrow_dy=pose[1]*self.arrowLen
                arrowPatch = patches.FancyArrow(pos[0],pos[1], arrow_dx, arrow_dy, color='yellow', width=0.05)
                self.visualization.add_patch(arrowPatch)
                self.arrowVisual[name]=arrowPatch
            target=np.array([self.leaders['Eigen']['Target'][0],self.leaders['Eigen']['Target'][1]])
            # self.plotQuiver(target)
            self.visualization.grid(False)
            self.goalVisual=self.visualization.plot(self.leaders['Eigen']['Target'][0],self.leaders['Eigen']['Target'][1],'rx')
            self.env.plotObstacles(self.visualization)
            self.plotNablaDelta()
            

        if self.agentSpawn==1 and self.graph.edges!=None:
            self.updatedEdges = self.cleanEdge(self.graph.edges)
            self.edgeData = {
                'edge': [],
                'distance': [],
                'timestep': []
            }
            self.nonEdgeData = {
                'edge': [],
                'distance': [],
                'timestep': []
            }
            # self.agentType=self.get_subclasses(agent,baseAgent)
            self.notEdges = self.find_non_edges(self.graph.edges)
            self.task=agentask(self.graph,self.leaders)
            self.populate(self.mode)
            self.y0=np.empty((0,1))
            for name in self.graph.names:
                self.y0=np.vstack((self.y0,self.agents[name].pos))
            self.figure,self.visualization=plt.subplots()
            # self.figure=self.visualization.get_figure()       
            self.workspacePatch=shapely.plotting.plot_polygon(self.env.workspace,add_points=False)
            self.visualization.add_patch(self.workspacePatch)
            # if self.worldType == 1:
                # buffer_patch = self.env.bufferPatch


            # form dictionary of agent position visual representations

            self.verticesVisual={name:patches.Circle(
                uv.col2tup(self.agents[name].pos),
                radius=0.2,
                label=name,
                color='purple' if name in self.leaders else 'orange',
                animated=True,
                ) for name in self.graph.names}

            # form dictionary of agent edges visual representations
            self.edgesVisual={edge:patches.Polygon(
                np.asarray(np.hstack((self.agents[edge[0]].pos,self.agents[edge[1]].pos)).T),
                closed=False,
                edgecolor='black',                                                              
                linestyle='-',
                animated=True,
                ) for edge in self.graph.edges}
            
            for edge in self.graph.edges:
                self.visualization.add_patch(self.edgesVisual[edge])
                # print(la.norm(self.edgesVisual[edge]))
                # self.vis2.plot()

            for name in self.graph.names:
                self.visualization.add_patch(self.verticesVisual[name])
            target=np.array([self.leaders['Eigen']['Target'][0],self.leaders['Eigen']['Target'][1]])
            
            self.goalVisual=self.visualization.plot(self.leaders['Eigen']['Target'][0],self.leaders['Eigen']['Target'][1],'rx')
            self.plotQuiver(target.reshape((2,1)))
            if self.LazyQ:
                titlePlot='Lazy PnP Controller'
            else:
                titlePlot='Contractive PnP Controller'
            self.env.plotObstacles(self.visualization)
            # self.visualization.set(title=titlePlot,xlabel='Workspace x-axis [m]', ylabel='Workspace y-axis [m]')
            self.visualization.grid(False)
            # self.figure.legend(loc='upper left',title='Agents')

        elif agentSpawn==1 and self.agentNum==1:
            for name,pos in self.stateWname:
                self.agents[name]=Agent(name, self.env, self,{'target': np.array([[9],[9]]), 'keepUpQ': False}, np.array(pos).reshape((2,1)))
            self.figure,self.visualization=plt.subplots()
            # self.figure=self.visualization.get_figure()       
            self.workspacePatch=shapely.plotting.plot_polygon(self.env.workspace,add_points=False)
            self.visualization.add_patch(self.workspacePatch)
            # form dictionary of agent position visual representations

            self.verticesVisual={name:patches.Circle(
                uv.col2tup(self.agents[name].pos),
                radius=0.2,
                label=name,
                color='purple' if name in self.leaders else 'orange',
                # animated=True,
                ) for name in self.graph.names}
            for name in self.graph.names:
                self.visualization.add_patch(self.verticesVisual[name])
            self.goalVisual=self.visualization.plot(self.leaders['Eigen']['Target'][0],self.leaders['Eigen']['Target'][1],'rx')
            
        elif not agentSpawn:
            self.figure,self.visualization=plt.subplots()
            # self.figure=self.visualization.get_figure()       
            self.workspacePatch=shapely.plotting.plot_polygon(self.env.workspace,add_points=False)
            self.visualization.add_patch(self.workspacePatch)
            #plot nav field
            target=np.array([9,9])
            self.plotQuiver(target)
            sys.exit()

    def plotQuiver(self,target):
        [xmin, ymin, xmax, ymax] = shapely.bounds(self.env.workspace)
        
        xArray=np.arange(xmin,xmax,0.7)
        yArray=np.arange(ymin,ymax,0.7)
        X,Y=np.meshgrid(xArray,yArray)
        lenX=X.shape
        U=np.zeros((lenX))
        V=np.zeros((lenX))
        maxX=lenX[0]
        for idx in range(maxX):
            for idy in range(maxX):
                
                state=np.array([X[idx, idy], Y[idx, idy]]).reshape((2,1))
                goal=np.array(target)
                if self.worldType == 1:
                    navV=self.env.navfStar(state,goal)
                elif self.worldType == 0:
                    navV=self.env.navfSphere(state,goal)
                elif self.worldType==2:
                    self.env.plotObstacles(self.visualization)
                    navV=self.env.polyNav(state,goal)
                U[idx,idy]=navV[0]
                V[idx,idy]=navV[1]

        self.visualization.quiver(X, Y, U, V)
        # print(self.env.nearestUnsafePoint(self.agents['Eigen'].pos))
        # add stuff to remove arrows inside obstacles
        # plt.show()
   
    def plotNablaDelta(self):
        [xmin, ymin, xmax, ymax] = shapely.bounds(self.env.workspace)
        
        # Use a larger step size to reduce arrow density
        xArray = np.arange(xmin, xmax, 1.0)
        yArray = np.arange(ymin, ymax, 1.0)
        X, Y = np.meshgrid(xArray, yArray)
        
        lenX = X.shape
        U = np.zeros(lenX)
        V = np.zeros(lenX)
        
        # Create a mask for points inside obstacles
        mask = np.ones(lenX, dtype=bool)
        
        maxX = lenX[0]
        for idx in range(maxX):
            for idy in range(maxX):
                state = np.array([X[idx, idy], Y[idx, idy]]).reshape((2,1))
                
                # Check if point is inside any obstacle
                point = shapely.geometry.Point(X[idx, idy], Y[idx, idy])
                is_inside_obstacle = False
                for i in range(self.env.obstacleNum):
                    center = self.env.obstacleCenters[i]
                    radius = self.env.obstacleRadii[i]
                    if point.distance(shapely.geometry.Point(center)) <= radius:
                        is_inside_obstacle = True
                        mask[idx, idy] = False
                        break
                
                if is_inside_obstacle:
                    U[idx, idy] = np.nan
                    V[idx, idy] = np.nan
                    continue
                    
                gradV = self.agents['Eigen'].nablaQDel(state)
                if gradV is None:
                    U[idx, idy] = np.nan  # Use np.nan instead of None
                    V[idx, idy] = np.nan
                else:
                    U[idx, idy] = gradV[0]
                    V[idx, idy] = gradV[1]
        
        # Calculate vector magnitudes for coloring
        magnitude = np.sqrt(U**2 + V**2)
        
        # Clear previous plots
        # self.visualization.clear()
        
        # Plot the vector field with improved visibility
        self.visualization.quiver(X, Y, U, V)               # Slight transparency
        
        # Add a colorbar to show magnitude scale
        # cbar = self.visualization.figure.colorbar(self.visualization.quiver(X, Y, U, V, magnitude, visible=False))
        # cbar.set_label('Gradient Magnitude')
        
        # Add streamlines for better flow visualization
        # self.visualization.streamplot(X, Y, U, V, 
        #                             density=1.0,          # Adjust density as needed 
        #                             color='white',        # Light color that contrasts with background
        #                             linewidth=0.8,        # Thin lines
        #                             arrowsize=0.8)        # Small arrows
        
        # Draw obstacles more clearly
        for i in range(self.env.obstacleNum):
            center = self.env.obstacleCenters[i]
            radius = self.env.obstacleRadii[i]
            circle = plt.Circle(center, radius, fill=True, color='white', edgecolor='black', alpha=0.8)
            self.visualization.add_patch(circle)
        
        # Draw workspace boundary
        workspace_boundary = self.env.workspace.exterior.coords
        x_coords, y_coords = zip(*workspace_boundary)
        self.visualization.plot(x_coords, y_coords, 'k-', linewidth=2)
        
        # Set plot properties
        self.visualization.set_aspect('equal')
        self.visualization.grid(False)
        self.visualization.set_title('Vector Field: Gradient to Nearest Unsafe Point')
        self.visualization.set_xlabel('X Position')
        self.visualization.set_ylabel('Y Position')
        
        # Save figure with higher resolution
        self.figure.tight_layout()
        self.figure.savefig("nablaDelta.svg", format="svg", dpi=300, bbox_inches='tight')
        
        # Show the plot
        plt.show()

    def pnpFlowMap(self, y, t):
        vertex_indices = self.graph.vertex_indices
        xStack = y.reshape(-1, 1)  # Reshape input to column vector
        n_agents = len(self.graph.names)
        dydt = np.zeros_like(xStack)

        for name in self.graph.names:
            idx = 2 * vertex_indices[name]
            myState = xStack[idx:idx+2]
            nbrs = self.neighbors(name)
            nbrIdx = [2 * vertex_indices[val] for val in nbrs]

            controlInput = np.zeros((2, 1))
            pnpSummand = np.zeros((2, 1))

            if self.task.taskList[name]['keepUpQ']:
                for nbr in nbrIdx:
                    nbrState = xStack[nbr:nbr+2]
                    navvec = self.env.navfSphere(myState, nbrState)
                    relpos = nbrState - myState
                    navxi = self.tension_func(la.norm(relpos)) * (la.norm(relpos)**2) / (relpos.T @ navvec + 1e-8)
                    pnpSummand += navxi * navvec

            targ = self.task.taskList[name]['target']
            if targ is None:
                controlInput = pnpSummand
            else:
                controlInput = pnpSummand + self.leaderGain * self.env.navfSphere(myState, targ.reshape((2, 1)))

            dydt[idx:idx+2] = controlInput

        return dydt.flatten()

    def pnpFlowMapsolo(self,y,t):
        # controlInput=np.zeros((2,1))
        myState=np.array((y))
        targ=np.array((self.task.taskList['Eigen']['target']))
        navvec=self.env.navfSphere(myState.reshape((2,1)),targ.reshape((2,1)))
        dydt=self.leaderGain*navvec
        return dydt.flatten()

    def neighbors(self,name):
        return self.graph.neighbors(name)
    
    def neighborPos(self,name,xStack):
        # returns a list of neighbor position for a given agent name
        ls=enumerate(self.graph.getrow(self.vertex_indices[name]).toarray()[0].tolist())
        return [self.names[x] for x,y in ls if y==1]    
    
    def populate(self,mode=0):
        if mode==0:
            self.spawnAgents()
        elif self.mode==1:
            for name,pos in self.stateWname:
                # print(name,self.task.taskList[name])
                self.agents[name]=Agent(name, self.env, self,self.task.taskList[name], np.array(pos).reshape((2,1)))
        elif self.mode==2:
            for name,pos in self.stateWname:
                self.agents[name]=unicycleAgent(name, self.env, self,self.task.taskList[name], np.array(pos).reshape((4,1)))
        elif self.mode==3:
            for name in self.graph.names:
                pos=self.env.generateRandPointPose()
                #somehow ensure the pos,pose are joined into a 4,1 like mode==2
                self.agents[name]=unicycleAgent(name, self.env, self,self.task.taskList[name], np.array(pos))
        else:
            raise Exception("Invalid network generation mode")
        
    def reportPosition(self,name):
        return self.agents[name].pos
        
    def spawnAgents(self):
    # go over dfs ordering of the vertices
        dfsOrder,dfsPredecessors=self.graph.dfs()        
        # for each vertex spawn a position
        for item,predName in zip(dfsOrder,dfsPredecessors):
            # if item has no predecessor then generate a random point in the workspace
            if predName == None:
                pt=self.env.generateRndPoint()
            # otherwise generate a random point within a distance of self.rcomm from the position of the predecessor
            else:
                parent_pos=self.agents[predName].pos
                pt=self.env.generateRndPoint(self.rsafe,parent_pos)
            # assign the generated pt to an agent object named item
            # here pt is a vertical np array
            self.agents[item]=Agent(item, self.env, self,self.task.taskList[item], pt)
        return self.agents
    
    def tension_func(self,s):
        # print(s)
        if s >= 0 and s<self.rsafe:
            return (1-self.LazyQ)*self.coopGain
        if s >= self.rsafe and s <= self.rcomm:
            return (1-self.LazyQ)*self.coopGain + self.omega*((s-self.rsafe)**(1+self.alpha))
        if s > self.rcomm:
                return 0
        # return 1        
    def dummyUpdate(self):
        # change all agents to rando positions 
        for name in self.graph.names:
            self.agents[name].pos=self.env.generateRndPoint()
 
    def pnpUpdate(self):
        vecs={}
        angs={}
        for name in self.graph.names:
            vecs[name]=self.agents[name].forward_control_input()
            angs[name]=self.agents[name].angular_control_input()
        # this is x_new=time*x_dot??
        for name in self.graph.names:
            # print('vec name')
            # print(vecs[name])
            self.agents[name].translatePos(self.dt*vecs[name])
            self.agents[name].translatePose(self.dt*angs[name])
    def updateVisualization(self):
        # updates the visualization data for vertices and edges
        self.timestart = round(self.timestart+self.dt,2)
        
        for name in self.graph.names:
            # print(self.agents[name].pos)
            pos = uv.col2tup(self.agents[name].pos)
            pose=uv.col2tup(self.agents[name].pose)
            self.verticesVisual[name].set(center=(pos[0],pos[1]))
            agentLinearVel=self.agents[name].v[0][0]
            arrow_dx=float(pose[0])*agentLinearVel
            arrow_dy=float(pose[1])*agentLinearVel
            # self.arrowVisual[name].set(x=pos[0], y=pos[1], dx=arrow_dx, dy=arrow_dy)
            self.arrowVisual[name].remove()
            arrowPatch = patches.FancyArrow(pos[0],pos[1], arrow_dx, arrow_dy, color='yellow', width=0.05, length_includes_head=True)
            self.visualization.add_patch(arrowPatch)
            self.arrowVisual[name] = arrowPatch
            # self.arrowVisual[name].set(pos[0],pos[1], arrow_dx, arrow_dy,width=None,head_width=None,head_length=None)

        if self.graph.edges != None:    
            for edge in self.graph.edges: 
                self.edgesVisual[edge].set_xy(np.asarray(np.hstack((self.agents[edge[0]].pos,self.agents[edge[1]].pos)).T))
            for edge in self.updatedEdges:
                dis = self.edgeDistance(self.agents[edge[0]].pos,self.agents[edge[1]].pos)
                self.update_edge_lengths(edge,dis,self.timestart)
            for nedge in self.notEdges:
                ndis = self.edgeDistance(self.agents[nedge[0]].pos,self.agents[nedge[1]].pos)
                self.update_nedge_lengths(nedge,ndis,self.timestart)
        if self.timestart>self.simTime:
            current_time = datetime.now()
            print(current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
            sys.exit()

        # self.figure.canvas.flush_events()

    def update_edge_lengths(self,edge, distance, timestep):
        self.edgeData['edge'].append(edge)
        self.edgeData['distance'].append(distance)
        self.edgeData['timestep'].append(timestep)
    def update_nedge_lengths(self,edge, distance, timestep):
        self.nonEdgeData['edge'].append(edge)
        self.nonEdgeData['distance'].append(distance)
        self.nonEdgeData['timestep'].append(timestep)
    
    def cleanEdge(self,list):
        seen = set()
        result = []

        for item in list:
            sorted_items = tuple(sorted(item))
            if sorted_items not in seen:
                seen.add(sorted_items)
                result.append(item)        
        return result
    
    def find_non_edges(self,edges):
        # Extract unique nodes from the list of edges
        nodes = set([node for edge in edges for node in edge])
        
        # Generate all possible edges (excluding self-loops)
        all_possible_edges = set(combinations(nodes, 2))
        
        # Convert edges to a set of tuples to account for both orientations
        existing_edges = set(tuple(sorted(edge)) for edge in edges)
        
        # Find non-edges by subtracting existing edges from all possible edges
        non_edges = all_possible_edges - existing_edges
        return non_edges

    def edgeDistance(self,xi,xj):
        return (la.norm(xi-xj))/self.rcomm
    
    def plotEdgeLenghts(self):
        plt.figure(figsize=(10, 6))
        for edge in set(self.edgeData['edge']):
            
            edge_data = [(d, t) for e, d, t in zip(self.edgeData['edge'], self.edgeData['distance'], self.edgeData['timestep']) if e == edge]
            distances, times = zip(*edge_data)
            plt.plot(times, distances, label=f'Edge {edge}')

        plt.axhline(y=1, color='r', linestyle='--', label='d=1 for norm/R')
        plt.xlabel('Time Step')
        plt.ylabel('Edge Length: ||xi-xj||/R')
        plt.title('Edge Lengths Over Time')
        plt.legend(loc='lower right')
        plt.grid(False)
        output_directory = 'output_plots'
        os.makedirs(output_directory, exist_ok=True)  # Create the directory if it doesn't exist
        filename = os.path.join(output_directory, 'edge_lengths_plot.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()  # Close the figure to free up memory

        print(f"Plot saved as {filename}")

    def plotNonEdgeLenghts(self):
        plt.figure(figsize=(10, 6))
        for nedge in set(self.nonEdgeData['edge']):
            
            nedge_data = [(d, t) for e, d, t in zip(self.nonEdgeData['edge'], self.nonEdgeData['distance'], self.nonEdgeData['timestep']) if e == nedge]
            distances, times = zip(*nedge_data)
            plt.plot(times, distances, label=f'nEdge {nedge}')
        # plt.axhline(y=1, color='r', linestyle='--', label='d=1 for norm/R')
        plt.xlabel('Time Step')
        plt.ylabel('Non edge Length: ||xi-xj||/R')
        plt.title('Non edge Lengths Over Time')
        plt.legend(loc='upper right')
        plt.grid(False)
        output_directory = 'output_plots'
        os.makedirs(output_directory, exist_ok=True)  # Create the directory if it doesn't exist
        filename = os.path.join(output_directory, 'nonEdge_lengths_plot.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()  # Close the figure to free up memory

        print(f"Plot saved as {filename}")
    
    def get_subclasses(module, base_class,agentTypedata):
        subclasses = []
        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj) and issubclass(obj, base_class) and obj != base_class:                
                subclasses.append(obj)
        #  
