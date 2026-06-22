#/bin/bash python3
from numpy import linalg as la
import numpy as np
import os,sys
import math
import agent
#from agent import Agent
from agent import fullyActuatedAgent
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
from graph_w_names import graph_w_names
from states import State

class netwk():

    def __init__(self,networkInfo,env):
        self.networkInfo=networkInfo # 6/15/26: Used to get definitions outside __init__ working; should probably be removed/changed later.
        self.netID=networkInfo['netID']
        self.simTime=networkInfo['networkInfo']['Duration']
        self.env=env
        
        # DWR 6/15/26: I remember we wanted to combine this with the agent calling loop, but calling agents.py requires self.graph, so this has to come first.
        self.agentNames=list(networkInfo['networkInfo']['Agents']['AgentInfo']) #form a list of the keys in the agent dictionary

        #form the list of edges and construct the network's .graph attribute (using the graphWNames class)
        self.graph=graph_w_names(self.agentNames,networkInfo['networkInfo']['Edges'])

        #form the agent list
        self.agents={}
        if networkInfo['networkInfo']['Agents']['Initialization']=='Provided':
            for name in networkInfo['networkInfo']['Agents']['AgentInfo']:
                self.agentTask=networkInfo['networkInfo']['Agents']['AgentInfo'][name]['Task']
                self.agents[name]=getattr(agent,networkInfo['networkInfo']['Agents']['AgentInfo'][name]['Type'])(name,self.env,self,self.agentTask,networkInfo['networkInfo']['Agents']['AgentInfo'][name]['State'])
        elif networkInfo['networkInfo']['Initialization']=='Random': ##deferred for later
            for name in networkInfo['Agents']['AgentInfo']: # stand-in for randomized SPAWNING code 
                self.agents[name]=getattr(agent,networkInfo['Agents']['AgentInfo'][name]['Type'])(**networkInfo['Agents']['AgentInfo'][name]['State'])
        
        #form a dictionary with additional data on the organization of the state vector
        # DWR: Will the state vector always contain the p vector, or only when the p vector is being used?
        self.stateVectorInfo={} # set up empty dictionary
        dictLoop=0 # loop counter setup
        for name in networkInfo['networkInfo']['Agents']['AgentInfo']:
            self.stateVectorInfo[name]=[dictLoop,dictLoop+self.agents[name].state.size()] # Assigns name key a list containing the first and last entry of that agent in the big state vector
            dictLoop=dictLoop+self.agents[name].state.size()
        self.networkStateSize=dictLoop

        #form the list of leaders # DWR 6/16/26: Could remove it
        #self.leaders = networkInfo['networkInfo']['Agents']['Leaders']

        #construct the network-level update

        #form the stateWname dict
        # self.stateWname = {} # empty dictionary setup
        # for name,stateVect in networkInfo['Agents']['AgentInfo']:
        #     self.stateWname[name] = networkInfo['Agents']['AgentInfo']


        #compute cooperation parameters, if relevant (?)
        self.pnpParameters=networkInfo['networkInfo']['pnpParameters']
        self.agentNum=self.graph.agentNum
        self.agents={name:None for name in self.graph.names}
        #self.stateWname=stateWname # I'm not sure we even need stateWname anymore. The only place it is currently used is in setting mode to 0 or 1.
        self.stateWname=1 # dummy value to get it running
        self.rcomm=self.pnpParameters['rcomm']
        self.rsafe=self.pnpParameters['rsafe']*self.pnpParameters['rcomm']
        self.alpha=self.pnpParameters['alpha']
        self.leaderGain=self.pnpParameters['leaderGain']
        self.LazyQ=bool(self.pnpParameters['Lazy'])
        self.coopGain=int(self.pnpParameters['coopGain'])
        self.m=1/self.pnpParameters['rsafe']
        #self.leaders=self.leaderList # may want to just use the initial self.leaderList for less confusion
        self.timestart=0.0
        self.worldType = 0 # dummy value to get running; Any code corresponding to this should be moved to Environment.py
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
        self.dt=0.01
        self.notEdges = self.find_non_edges(self.graph.edges)
        if self.stateWname:
            self.mode=1
        else:
            self.mode=0
        
        ### compute some of the pnp parameters

        if self.graph.edges!=None:

            self.numEdges=len(self.graph.edges)
        
            if self.LazyQ:
                self.rsafeStar=self.rsafe+(self.rcomm-self.rsafe)/(self.numEdges**(1/(2+self.alpha)))

                self.omega=self.coopGain        
            else:
                self.rsafeStar=self.rsafe
                self.omega=self.coopGain*((2+self.alpha)/(2*self.rcomm**(1+self.alpha)))*(self.m**(1+self.alpha)*(self.numEdges-self.m**2))/((self.m-1)**(2+self.alpha))
        self.interiorPt=np.array([-5,-5])
        # if experiment requires agents
        
        # 6/9/2026: Starting from here & below, need to rewrite the ntwork visualization code into generic code

        self.agentSpawn=1 # dummy value to replace input from __init__. Should be rewritten
        if self.agentSpawn and self.graph.edges!=None:
            self.task=agentask(self.graph,networkInfo['networkInfo']['Agents']['AgentInfo']) # 6/16/2026 DWR: Going with the task-oriented instead of leader-based approach
            self.populate(self.mode)
            self.y0=np.empty((0,1))
            for name in self.graph.names:
                self.y0=np.vstack((self.y0,self.agents[name].state.q[0][0])) # DWR 6/17/2026: We should remove this, since we intend to use stateVectorInfo
            self.figure,self.visualization=plt.subplots()
            # self.figure=self.visualization.get_figure()       
            self.workspacePatch=shapely.plotting.plot_polygon(self.env.workspace,add_points=False)
            self.visualization.add_patch(self.workspacePatch)
            # if self.worldType == 1:
                # buffer_patch = self.env.bufferPatch


            # form dictionary of agent position visual representations

            self.verticesVisual={name:patches.Circle(
                #uv.col2tup(self.agents[name].pos),
                uv.col2tup(np.matrix(self.agents[name].state.q)), #not sure if the matrix conversion is the best way to do this
                radius=0.2,
                label=name,
                #color='purple' if name in self.leaders else 'orange',
                color='purple',
                animated=True,
                ) for name in self.graph.names}

            # form dictionary of agent edges visual representations
            self.edgesVisual={edge:patches.Polygon(
                np.asarray(np.hstack((np.array(self.agents[self.graph.indexToVertex[edge[0]]].state.q),np.array(self.agents[self.graph.indexToVertex[edge[1]]].state.q)))),
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
            target=np.array((networkInfo['networkInfo']['networkTask']['Goals']['Goal1']))
            
            self.goalVisual=self.visualization.plot(target[0],target[1],'rx')
            self.plotQuiver(target.reshape((2,1)))
            if self.LazyQ:
                titlePlot='Lazy PnP Controller'
            else:
                titlePlot='Contractive PnP Controller'
            self.env.plotObstacles(self.visualization)
            # self.visualization.set(title=titlePlot,xlabel='Workspace x-axis [m]', ylabel='Workspace y-axis [m]')
            self.visualization.grid(False)
            # self.figure.legend(loc='upper left',title='Agents')




        elif self.agentSpawn and self.agentNum==1: # Needs to be cleaned up
            for agentName in networkInfo['Agents']['AgentInfo']: # 6/10/26 Dan code with minor edits by Davy
                self.agents[agentName]=getattr(agent,agentName['Type'])(**agentName['State'])
            # for name,pos in self.stateWname:
            #     self.agents[name]=Agent(name, self.env, self,{'target': np.array([[9],[9]]), 'keepUpQ': False}, np.array(pos).reshape((2,1)))
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
            self.goalVisual=self.visualization.plot(self.leaders['Zoe']['Target'][0],self.leaders['Zoe']['Target'][1],'rx',markersize=12, markeredgewidth=3)
            target=np.array([9,9])
            self.plotQuiver(target)
            self.env.plotObstacles(self.visualization)
            plt.grid(False)
            # plt.show()

            
        elif not self.agentSpawn:
            self.figure,self.visualization=plt.subplots()
            # self.figure=self.visualization.get_figure()       
            self.workspacePatch=shapely.plotting.plot_polygon(self.env.workspace,add_points=False)
            self.visualization.add_patch(self.workspacePatch)
            #plot nav field
            target=np.array([9,9])
            self.plotQuiver(target)
            sys.exit()


    def formNetworkStateVector(self):
        x=np.zeros((1,self.networkStateSize))
        for name in self.agentNames:
            a,b=self.stateVectorInfo[name]
            x[0,a:b+1]=x[0,a:b+1]+np.array(self.agents[name].state.flatten()).T
        return x

    def updateNetworkState(self,ns):
        #distribute the flattened network state, denoted ns, to all the agents in the network
        for name in self.agents:
            a,b=self.stateVectorInfo[name]
            self.agents[name].update(ns[a:b,0])
        #return None

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
                state=State(np.array([X[idx, idy], Y[idx, idy]]).reshape((2,1)))
                goal=State(np.array(target))
                if self.worldType == 1:
                    navV=self.env.navfStar(goal,state)
                elif self.worldType == 0:
                    navV=self.env.nav(goal,state)
                U[idx,idy]=navV[0,0]
                V[idx,idy]=navV[1,0]

        self.visualization.quiver(X, Y, U, V)

        # add stuff to remove arrows inside obstacles
        # plt.show()


    def FlowMap(self, y, t):
        xStack = y.reshape(-1, 1)  # Reshape input to column vector (ODE solver inputs and outputs row state vectors)
        dydt = np.zeros_like(xStack)

        for name in self.agents:
            # obtain the state of the current agent in the loop
            a,b=self.stateVectorInfo[name]
            dydt[a:b,0]=(self.agents[name].clientOutputSim(xStack)).flatten()
        return dydt.T

    # def pnpFlowMap(self,y,t):

    #     vertex_indices=self.graph.vertex_indices
    #     # print(vertex_indices)
    #     xStack=y
    #     print(y)
    #     controlInput=np.zeros((2,1))
    #     pnpSummand=np.zeros((2,1))
    #     dydt=np.zeros((len(xStack),1))
    #     for name in self.graph.names:
    #         # myState=self.nameMap(name,xStack)
    #         myState=xStack[2*vertex_indices[name]:(2*vertex_indices[name])+2]
    #         nbrs=self.neighbors(name)
    #         nbrIdx=np.zeros(len(nbrs),dtype=np.int64)
    #         for i,val in enumerate(nbrs):        
    #             nbrIdx[i]=2*vertex_indices[val]
    #         if self.task.taskList[name]['keepUpQ']:
    #             for nbr in nbrIdx:
    #                 navvec=self.env.navfSphere(myState.reshape((2,1)),xStack[nbr:nbr+2].reshape((2,1)))
    #                 relpos=xStack[nbr:nbr+2]-myState
    #                 navxi=self.tension_func(la.norm(relpos))*(la.norm(relpos)**2)/(0.+(relpos.T@navvec))
    #                 pnpSummand=pnpSummand+navxi*navvec
    #                 # print(pnpSummand)
    #         targ=self.task.taskList[name]['target']
    #         if targ is None:
    #             controlInput=controlInput+pnpSummand
    #         else:                
    #             controlInput=(controlInput+self.leaderGain*self.env.navfSphere(myState.reshape((2,1)),targ.reshape((2,1))))
    #             # print(targ,myState)  
    #         dydt[vertex_indices[name]:vertex_indices[name]+2]=np.array((controlInput))
    #     return dydt.flatten(order="F")
    

    def pnpFlowMapsolo(self,y,t):
        # controlInput=np.zeros((2,1))
        myState=np.array((y))
        targ=np.array((self.task.taskList['Zoe']['target']))
        navvec=self.env.nav(myState.reshape((2,1)),targ.reshape((2,1))) # Edit navfsphere according to new nav-per-environment system
        dydt=self.leaderGain*navvec
        return dydt.flatten()
    
    def pnpFlowMapsoloUnicycle(self,y,t):
        # controlInput=np.zeros((2,1))
        myState=np.array((y))
        targ=np.array((self.task.taskList['Zoe']['target']))
        navvec=self.env.nav(myState.reshape((2,1)),targ.reshape((2,1))) # Edit navfsphere according to new nav-per-environment system
        dydt=self.leaderGain*navvec
        return dydt.flatten()

    def neighbors(self,name):
        return self.graph.neighbors(name)
    
    def neighborPos(self,name,xStack):
        # returns a list of neighbor position for a given agent name
        ls=enumerate(self.graph.getrow(self.graph.vertexIndices[name]).toarray()[0].tolist())
        return [self.names[x] for x,y in ls if y==1]    
    
    def populate(self,mode=0):
        if mode==0:
            self.spawnAgents()
        elif self.mode==1:
            for name in self.agentNames: # 6/10/26 Dan code with edits by Davy. May need changes to reflect its usage in populate
                self.agents[name]=getattr(agent,self.networkInfo['networkInfo']['Agents']['AgentInfo'][name]['Type'])(name,self.env,self,self.agentTask,self.networkInfo['networkInfo']['Agents']['AgentInfo'][name]['State']) 
            # for name,pos in self.stateWname: # code replaced by 6/10/26 Dan and Davy code
            #     # print(name,self.task.taskList[name])
            #     #self.agents[name]=Agent(name, self.env, self,self.task.taskList[name], np.array(pos).reshape((2,1)))
            #     self.agents[name]=Agent(name, self.env, self,self.task.taskList[name], np.array(pos).reshape((2,1)))
        else:
            raise Exception("Invalid network generation mode")
        
    def reportPosition(self,name):
        return self.agents[name].state.q
        
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
            self.agents[item]=getattr(agent,self.networkInfo['Agents']['AgentInfo'][item]['Type'])
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
        for name in self.graph.names:
            vecs[name]=self.agents[name].pnp()   

        for name in self.graph.names:
            self.agents[name].translatePos(self.dt*vecs[name])
    
    def updateVisualization(self):
        # updates the visualization data for vertices and edges
        self.timestart = round(self.timestart+self.dt,2)
        
        for name in self.graph.names:
            self.verticesVisual[name].set(center=uv.col2tup(self.agents[name].pos))

        if self.graph.edges != None:    
            for edge in self.graph.edges: 
                self.edgesVisual[edge].set_xy(np.asarray(np.hstack((self.agents[edge[0]].pos,self.agents[edge[1]].pos)).T))
        for edge in self.updatedEdges:
            dis = self.edgeDistance(self.agents[edge[0]].pos,self.agents[edge[1]].pos)
            print(dis)
            self.update_edge_lengths(edge,dis,self.timestart)
        for nedge in self.notEdges:
            ndis = self.edgeDistance(self.agents[nedge[0]].pos,self.agents[nedge[1]].pos)
            self.update_nedge_lengths(nedge,ndis,self.timestart)
        if self.timestart>self.simTime:
            self.plotEdgeLenghts()
            self.plotNonEdgeLenghts()
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
        if list is None:
            return []
        seen = set()
        result = []

        for item in list:
            sorted_items = tuple(sorted(item))
            if sorted_items not in seen:
                seen.add(sorted_items)
                result.append(item)        
        return result
    
    def find_non_edges(self, edges):
        if edges is None:
            # If edges is None, all possible pairs are non-edges
            nodes = set(self.graph.names)  # Assuming self.names contains all node names
            return set(combinations(nodes, 2))
        else:
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
        # plt.legend(loc='upper right')
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
        if agentTypedata:
            filtered_subclasses = [cls for cls in subclasses if cls.__name__ in agentTypedata]
            return filtered_subclasses
        else:
            return subclasses