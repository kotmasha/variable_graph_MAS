#/bin/bash python3
from numpy import linalg as la
import numpy as np

import math
import agent
from agent import baseAgent
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import shapely
import universal as uv
# from Navigation import radialNav as navField
from agentTask import agentask
import inspect

class netwk():

    def __init__(self,netID,graphWithNames,env,leaders,pnpParameters,stateWname=None):
        self.graph=graphWithNames
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
        self.agentType=self.get_subclasses(agent,baseAgent)
        self.dt=0.001
        if self.stateWname:
            self.mode=1
        else:
            self.mode=0
        self.leaders=leaders
        self.m=1/pnpParameters['rsafe']
        self.numEdges=len(self.graph.edges)
        self.interiorPt=np.array([-5,-5])
        if self.LazyQ:
            self.rsafeStar=self.rsafe+(self.rcomm-self.rsafe)/(self.numEdges**(1/(2+self.alpha)))

            self.omega=self.coopGain        
        else:
            self.rsafeStar=self.rsafe
            self.omega=self.coopGain*((2+self.alpha)/(2*self.rcomm**(1+self.alpha)))*(self.m**(1+self.alpha)*(self.numEdges-self.m**2))/((self.m-1)**(2+self.alpha))


        self.task=agentask(self.graph,self.leaders)
        self.populate(self.mode)
        self.y0=np.empty((0,1))
        for name in self.graph.names:
            self.y0=np.vstack((self.y0,self.agents[name].pos))

        self.figure,self.visualization=plt.subplots()
        # self.fig2,self.vis2=plt.subplots()

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

        # form dictionary of agent edges visual representations
        self.edgesVisual={edge:patches.Polygon(
            np.asarray(np.hstack((self.agents[edge[0]].pos,self.agents[edge[1]].pos)).T),
            closed=False,
            edgecolor='black',                                                              
            linestyle='-',
            # animated=True,
            ) for edge in self.graph.edges}
        
        for edge in self.graph.edges:
            self.visualization.add_patch(self.edgesVisual[edge])
            # print(la.norm(self.edgesVisual[edge]))
            # self.vis2.plot()

        for name in self.graph.names:
            self.visualization.add_patch(self.verticesVisual[name])
        self.goalVisual=self.visualization.plot(self.leaders['Zoe']['Target'][0],self.leaders['Zoe']['Target'][1],'rx')

        # n=env.navfSphere(self.agents['Zoe'].pos,np.array([14,14]).T)

    def pnpFlowMap(self,y,t):
        out=np.empty((0,1))
        vertex_indices=self.graph.vertex_indices
        xStack=y
        controlInput=np.zeros((2,1))
        pnpSummand=np.zeros((2,1))
        dydt=np.zeros((len(xStack),1))
        for name in self.graph.names:
            # myState=self.nameMap(name,xStack)
            myState=xStack[2*vertex_indices[name]:(2*vertex_indices[name])+2]
            nbrs=self.neighbors(name)
            nbrIdx=np.zeros(len(nbrs),dtype=np.int64)
            for i,val in enumerate(nbrs):        
                nbrIdx[i]=2*vertex_indices[val]
            if self.task.taskList[name]['keepUpQ']:
                for nbr in nbrIdx:
                    navvec=self.env.navfSphere(xStack[nbr:nbr+2].reshape((2,1)),myState.reshape((2,1)))
                    relpos=xStack[nbr:nbr+2]-myState
                    navxi=self.tension_func(la.norm(relpos))*(relpos.T@relpos)/(0.+(relpos.T@navvec))
                    pnpSummand=pnpSummand+navxi*navvec
            targ=self.task.taskList[name]['target']
            if targ is None:
                controlInput=controlInput+pnpSummand
            else:                
                controlInput=(controlInput+self.leaderGain*self.env.navfSphere(targ.reshape((2,1)),myState.reshape((2,1))))
                print(targ,myState)  
            dydt[vertex_indices[name]:vertex_indices[name]+2]=np.array((controlInput))
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
                self.agents[name]=self.agentType[name](name, self.env, self,self.task.taskList[name], np.array(pos).reshape((2,1)))
        else:
            raise Exception("Invalid network generation mode")
        
    def reportPosition(self,name):
        return self.agents[name].pos
    
    def spawnAgents(self):
    # go over dfs ordering of the vertices
        dfsOrder,dfsPredecessors=self.graph.dfs()        
        # for each vertex spawn a position
        for item,predName in zip(dfsOrder,dfsPredecessors):
        # for item,predName in self.graph.dfs():
            # if item has no predecessor then generate a random point in the workspace
            if predName == None:
                pt=self.env.generateRndPoint()
            # otherwise generate a random point within a distance of self.rcomm from the position of the predecessor
            else:
                parent_pos=self.agents[predName].pos
                pt=self.env.generateRndPoint(self.rsafe,parent_pos)
            # assign the generated pt to an agent object named item
            # here pt is a vertical np array
            self.agents[item]=self.agentType[item](item, self.env, self,self.task.taskList[item], pt)
        
        

        return self.agents
    
    def tension_func(self,s):
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
        for name in self.graph.names:
            self.verticesVisual[name].set(center=uv.col2tup(self.agents[name].pos))
            
        for edge in self.graph.edges:
            self.edgesVisual[edge].set_xy(np.asarray(np.hstack((self.agents[edge[0]].pos,self.agents[edge[1]].pos)).T))
        plt.show()
        return None    

    def get_subclasses(module, base_class,agentTypedata):
        subclasses = []
        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj) and issubclass(obj, base_class) and obj != base_class:                
                subclasses.append(obj)
        # for 
