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
    def __init__(self,networkInfo,env,visualDict):
        self.networkInfo=networkInfo # 6/15/26: Used to get definitions outside __init__ working; should probably be removed/changed later.
        self.netID=networkInfo['netID']
        self.simTime=networkInfo['networkInfo']['Duration']
        self.env=env
        # visualDict is a dictionary of axes objects that can accept an add_patch command
        
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
        self.time=0.0
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

        if len(self.graph.edges)!=0:

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

        self.populate(self.mode) # Input and implementation TBD

        # form dictionary of agent position visual representations
        self.verticesVisual={} # If line 132 fails, try double braces like {{}}
        self.edgesVisual={}

        for vis in visualDict:
            for name in self.graph.names:
                #for key in self.agents[name].visualizationKeyList:
                self.verticesVisual[name]={}
                for key in self.agents[name].visualization:
                    vertexPatch=self.agents[name].visualization[key]
                    self.verticesVisual[name][key]=vertexPatch
                    visualDict[vis].add_patch(vertexPatch)
            for edge in self.graph.edges:
                edgePatch=patches.Polygon(
                    np.asarray(np.hstack((np.array(self.agents[self.graph.indexToVertex[edge[0]]].state.q),np.array(self.agents[self.graph.indexToVertex[edge[1]]].state.q)))),
                    closed=False,
                    edgecolor='black',                                                              
                    linestyle='-',
                    animated=True,
                    )
                self.edgesVisual[edge]=edgePatch
                visualDict[vis].add_patch(edgePatch)
        self.visualDict=visualDict # for export
        
        #DWR 7/17/2026: put all goals into the visualization
        #self.target=np.array((networkInfo['networkInfo']['networkTask']['Goals']['Goal1']))

        #self.env.plotObstacles(self.visualization)
        # self.visualization.set(title=titlePlot,xlabel='Workspace x-axis [m]', ylabel='Workspace y-axis [m]')
        #self.visualization.grid(False)
        # self.figure.legend(loc='upper left',title='Agents')

    def tick(self,t=None):
        if t is None:
            self.setTime(self.time+self.dt)
        else:
            self.setTime(t)

    def setTime(self,t):
        self.time=t

    def formNetworkStateVector(self):
        x=np.zeros((1,self.networkStateSize))
        for name in self.agentNames:
            a,b=self.stateVectorInfo[name]
            x[0,a:b]=x[0,a:b]+np.array(self.agents[name].state.flatten()).reshape(1,-1)
        return x

    def updateNetworkState(self,ns):
        #distribute the flattened network state, denoted ns, to all the agents in the network
        for name in self.agents:
            a,b=self.stateVectorInfo[name]
            self.agents[name].update(ns[a:b,0])
        #return None

    def headingPlot(self):
        X,Y,U,V=np.zeros(len(self.agentNames))
        for i in range(0,len(self.agentNames)):
            X[i]=self.networkInfo['Agents']['AgentInfo'][self.agentNames[i]]['q'][0]
            Y[i]=self.networkInfo['Agents']['AgentInfo'][self.agentNames[i]]['q'][1]
            U[i]=self.networkInfo['Agents']['AgentInfo'][self.agentNames[i]]['p'][0]
            V[i]=self.networkInfo['Agents']['AgentInfo'][self.agentNames[i]]['p'][1]
        return X,Y,U,V


    def FlowMap(self, y, t):
        xStack = y.reshape(-1, 1)  # Reshape input to column vector (ODE solver inputs and outputs row state vectors)
        dydt = np.zeros_like(xStack)

        for name in self.agents:
            # obtain the state of the current agent in the loop
            a,b=self.stateVectorInfo[name]
            dydt[a:b,0]=(self.agents[name].clientOutputSim(np.matrix(xStack))).flatten()
        return dydt.flatten() # DWR 6/23/2026: Odeint threw a "ndim=2" error until I flattened it
    
    # Identical to FlowMap, but with t and y swapped in input. Created to accomodate scipy solve_ivp instead of scipy odeint
    def FlowMapSwapInput(self,t,y):
        xStack = y.reshape(-1, 1)  # Reshape input to column vector (ODE solver inputs and outputs row state vectors)
        dydt = np.zeros_like(xStack)

        for name in self.agents:
            # obtain the state of the current agent in the loop
            a,b=self.stateVectorInfo[name]
            dydt[a:b,0]=(self.agents[name].clientOutputSim(np.matrix(xStack))).flatten()
        return dydt.flatten()

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
            for name in self.networkInfo['networkInfo']['Agents']['AgentInfo']:
                self.agentTask=self.networkInfo['networkInfo']['Agents']['AgentInfo'][name]['Task']
                self.agents[name]=getattr(agent,self.networkInfo['networkInfo']['Agents']['AgentInfo'][name]['Type'])(name,self.env,self,self.agentTask,self.networkInfo['networkInfo']['Agents']['AgentInfo'][name]['State'])
            # for name,pos in self.stateWname: # code replaced by 6/10/26 Dan and Davy code
            #     # print(name,self.task.taskList[name])
            #     #self.agents[name]=Agent(name, self.env, self,self.task.taskList[name], np.array(pos).reshape((2,1)))
            #     self.agents[name]=Agent(name, self.env, self,self.task.taskList[name], np.array(pos).reshape((2,1)))
        else:
            raise Exception("Invalid network generation mode")
        
    def reportPosition(self,name):
        return self.agents[name].state.q

    def reportState(self,name):
        return self.agents[name].state
        
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
 
    def pnpUpdate(self,networkState=None): # DWR 7/2/2026: "Simulation mode" written, still needs "robot mode"
        networkState=np.matrix(networkState).T # state.q is vertical matrix, horizontal array networkState causes problems
        for name in self.graph.names:
            a,b=self.stateVectorInfo[name]
            self.agents[name].setState(networkState[a:b])
    
    # def updateVisualization(self,timeStamp): # DWR 7/2/2026: Do we need timeStamp?
    def updateVisualization(self):   
        for name in self.verticesVisual:
            for item in self.verticesVisual[name]:
                # Safe polygon
                if item=='safePolygon':
                    self.verticesVisual[name][item].set_path(self.agents[name].safePolyViz())
                elif item=='goalProjection':
                    self.verticesVisual[name][item].set(center=uv.col2tup(self.agents[name].goalProjViz()))
                # Generic type stuff begins here; Consider going entirely by the item name
                elif type(self.verticesVisual[name][item])==patches.Arrow:
                    self.verticesVisual[name][item].set_data(x=self.agents[name].state.q[0].item(),y=self.agents[name].state.q[1].item(),dx=np.cos(self.agents[name].state.angle),dy=np.sin(self.agents[name].state.angle))
                elif type(self.verticesVisual[name][item])==patches.Circle:
                    self.verticesVisual[name][item].set(center=uv.col2tup(self.agents[name].state.q))
                elif type(item)==patches.PathPatch:
                    self.verticesVisual[name][item]
                else:
                    raise Exception(f"Need to add If statement for patch type: {type(self.verticesVisual[name][item])}")

        if len(self.graph.edges) != 0:    
            for edge in self.graph.edges: 
                self.edgesVisual[edge].set_xy(np.asarray(np.hstack((self.agents[self.graph.names[edge[0]]].state.q,self.agents[self.graph.names[edge[1]]].state.q)).T))
        for edge in self.updatedEdges:
            dis = self.edgeDistance(self.agents[self.graph.names[edge[0]]].state.q,self.agents[self.graph.names[edge[1]]].state.q)
            # print(dis)
            self.update_edge_lengths(edge,dis,self.time)
        for nedge in self.notEdges:
            ndis = self.edgeDistance(self.agents[self.graph.names[nedge[0]]].state.q,self.agents[self.graph.names[nedge[1]]].state.q)
            self.update_nedge_lengths(nedge,ndis,self.time)
        if self.time>self.simTime:
            self.plotEdgeLenghts()
            self.plotNonEdgeLenghts()
            current_time = datetime.now()
            print(current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
            sys.exit() # DWR 7/1/2026: Look into this, see what is going on with this if block

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