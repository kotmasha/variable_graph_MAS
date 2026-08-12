#/bin/bash python3

import sys
import os
import math
import matplotlib.patches as patches
import matplotlib.path as matPaths
from numpy import linalg as la
import numpy as np
import qpsolvers
import random
# from scipy.sparse.csgraph import depth_first_order
from scipy.spatial import HalfspaceIntersection
import shapely
from states import State, State2ndOrder, State2ndOrdRadian
import universal as uv
from myGeometryTools import skewJ
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from tf_transformations import quaternion_from_euler

# Future goal: Design agent class as a "parent" with minimal functionality, and re-structure for a fully-actuated and a unicycle agent separately.




class Agent():
    def __init__(self,name,env,network,task,state):
        # super().__init__('baseAgent_node')
        self.name=name
        self.env=env
        self.network=network
        self.state=state # 6/9/2026 State will always have an attribute called "q" indicating position. It could have others too.
        self.neighbors=network.neighbors(name)
        self.task=task
        self.plots=network.networkInfo['networkInfo']['Agents']['AgentInfo'][name]['Plots'] # List of strings, which are things to plot

        self.visualization={}
        # UNCOMMENT THIS WHEN READY; IT IS INTENDED TO STAY HERE
        self.visualization['vertex']=patches.Circle(
            uv.col2tup(self.state.q),
            radius=0.15,
            label=name,
            color='orange',
            animated=True, # ENABLE IF CREATING VIDEO
            )
        if 'safePolygon' in self.plots: # See safePolyViz for more info below
            goal=np.matrix(self.network.networkInfo['networkInfo']['networkTask']['Goals'][self.task['Target']]).T
            pos=np.array([[self.state.q[0].item()],[self.state.q[1].item()]]) # halfspace needs shape (2,), and npmatrix flatten doesn't work right
            poly=HalfspaceIntersection(np.hstack((self.env.safetyMatrixExtended(pos),-self.env.safetyCoefficientsNoGoal(pos))),interior_point=np.ndarray.flatten(pos)) #computing the vertices of the safety polygon        

            verticesTemp=np.subtract(poly.intersections,pos.T)
            vertexSorter=np.argsort(np.arctan2(verticesTemp[:,0],verticesTemp[:,1]))

            poly=shapely.geometry.Polygon(poly.intersections[vertexSorter])
            self.visualization['safePolygon']=shapely.plotting.plot_polygon(poly,add_points=False,animated=True)

            #safePolygon other options begin here
            if 'goalProjection' in self.plots['safePolygon']:
                #Find a quicker way to do this than calling Nav again
                self.visualization['goalProjection']=patches.Circle(
                    uv.col2tup(self.state.q+self.env.nav(goal,pos)),
                    radius=0.1,
                    label=name,
                    color='red',
                    animated=True, # ENABLE IF CREATING VIDEO
                    )


        # self.addEgde=self.create_service(AddEdge,'AddEdge',)
        # #Define parameters here
        # self.declare_parameter('x_init', self.pos[0])
        # self.declare_parameter('y_init', self.pos[1])
        # self.declare_parameter('name', self.name)
        
    def own_state(self,networkState=None): # returns the agent's state, or, if a networkState is provided, then returns the agent's component in it as a state of the same subclass
        if networkState is None:
            return self.state
        else:
            a,b=self.network.stateVectorInfo[self.name]
            return self.state.__class__(networkState[a:b,0])

    def pollNeighborsStates(self):
        if self.neighbors==None:
            return None
        else:
            return {name:self.network.reportState(name) for name in self.neighbors}
            # if networkState is None: # If no networkstate given, return dictionary of the states of all agents 
            #     return {name:self.network.reportState(name) for name in self.neighbors}
            # else: # If networkstate is given, return dictionary of states of all agents according to networkState vector
            #     d={}
            #     for name in self.neighbors:
            #         a,b=self.network.stateVectorInfo[name]
            #         d[name]=networkState[a:b,0]
            #     return d
    
    def navf(self,goal,inputPos=None): # both goal and inputPos are position vectors (column matrices)
        if inputPos is None:
            return self.env.nav(goal,self.state.pos())
        else:
            return self.env.nav(goal,inputPos)

    def translatePos(self,vec):
        self.state.q=self.state.q+vec # Think about this & state/2ndorder class
    
    def setState(self,vec): # update the agent's state using vec
        self.state.update(vec)
    
    def dynamics(self,controlInput,inputState=None): # DWR 6/17/2026: I think this is supposed to give a route to the controller computing for each agent type
        if inputState is None:
            inputState=self.state # Workaround for default value
        #some computation here using self.state and controlInput 
        return 0. # 0 is default dynamics value

    def computeController(self):
        controller=np.zeros(self.state.q.size) # Default controller is null
        return controller

    def clientOutputRobot(self): # For robot
        return self.computeController(self)
    
    def clientOutputSim(self,networkState): # for ODE simulation, networkState is a column np.array vector
        return self.dynamics(self.computeController(networkState),self.own_state(networkState))

    def safePolyViz(self): # for use in animation
        pos=np.array([[self.state.q[0].item()],[self.state.q[1].item()]]) # halfspace needs shape (2,), and npmatrix flatten doesn't work
        if (np.min(pos)<0) or (np.max(pos)>10): # Temporary hard-code to bugfix unicycle vasilos; should be improved if used more, otherwise removed
            poly=HalfspaceIntersection(np.hstack((self.env.safetyMatrixExtended(pos)[0:-4],-self.env.safetyCoefficientsNoGoal(pos)[0:-4])),interior_point=np.ndarray.flatten(pos))
        else:
            poly=HalfspaceIntersection(np.hstack((self.env.safetyMatrixExtended(pos),-self.env.safetyCoefficientsNoGoal(pos))),interior_point=np.ndarray.flatten(pos)) #computing the vertices of the safety polygon        

        # halfspaceIntersection doesn't sort the vertices for us, so we do it here
        verticesTemp=np.subtract(poly.intersections,pos.T) # move interior point to 0,0 for CCW sorting
        vertexSorter=np.argsort(np.arctan2(verticesTemp[:,0],verticesTemp[:,1])) # indices of poly vertices sorted by CCW order
        polyVertices=poly.intersections[vertexSorter]
        
        return matPaths.Path(np.vstack((polyVertices,polyVertices[0])),closed=True)

    def goalProjViz(self):
        goal=np.matrix(self.network.networkInfo['networkInfo']['networkTask']['Goals'][self.task['Target']]).T
        return (self.state.q + self.env.nav(goal,self.state.q))

    
        
        
    
class fullyActuatedAgent(Agent):
    def __init__(self,name,env,network,task,state):
        state=State(np.array(state['q']).T)
        super().__init__(name,env,network,task,state)
    
    def dynamics(self,controlInput,inputState=None): # implementation of xdot=f(x,u)=u
        if inputState is None:
            inputState=self.state
        return controlInput
    
    def computeController(self,virtualState=None):  # See algorithm 4.2 in thesis for more info.
        states=self.pollNeighborsStates() # Obtain a list of neighbors' states and reduce them to positions
        if virtualState is None:
            positions={name:states[name].pos() for name in states}
        else:
            positions={}
            for name in states:
                a,b=self.network.stateVectorInfo[name]
                positions[name]=(states[name].pos(virtualState[a:b,0])).reshape(-1,1)

        my_pos=self.own_state(virtualState).pos()
        # Prepare "empty" control input vector
        controlInput=np.zeros_like(my_pos)
        # Prepare "empty" network interaction component
        pnpSummand=np.zeros_like(my_pos)
        # Accumulating the network interaction component
        if self.task['KeepUpQ']:
            for name in self.neighbors: # 6/9/2026: Consider rewriting to iterate through the entries of the positions dictionary
                navvec=self.navf(positions[name],my_pos)
                relpos=positions[name]-my_pos # 
                navxi=self.network.tension_func(la.norm(relpos))*(la.norm(relpos)**2)/(0.+(relpos.T@navvec))
                pnpSummand=pnpSummand+navvec*navxi #reordered navvec and navxi, it gave a not aligned error since navxi is a 1x1 array
        
        # Calculate the navigation-to-goal component
        if 'Target' in self.task:
            targ=np.matrix(np.reshape(self.network.networkInfo['networkInfo']['networkTask']['Goals'][self.task['Target']],shape=np.shape(my_pos)))
            targNav=self.navf(targ,my_pos) 
            controlInput=controlInput+self.network.leaderGain*targNav

            
            
        # Combine the target component with the network interaction component        
        controlInput=controlInput+pnpSummand
        return controlInput
    

class unicycleAgent(Agent): #Will use 2nd order state from states.py
    def __init__(self,name,env,network,task,state):
        state=State2ndOrder(np.vstack(((np.matrix(state['q'])).T,(np.matrix(state['p'])).T)))
        super().__init__(name,env,network,task,state)
        # self.pose=yaw

    def dynamics(self,controlInput,inputState=None):
    # Proposed inputs: controlInput is
        if inputState is None:
            inputState=self.state
        qdot=controlInput[0]*inputState.p # Matches notation in the unicycle paper
        pdot=controlInput[1]*skewJ*inputState.p
        return np.vstack((qdot,pdot))

    # def nav(self,goal): # DWR 6/23/2026: Not sure if it needs its own navigation function classed here or not
    #     return self.env.nav(goal,self.pos)
    
    def pushField(self,pos): # To be used in the controller
        #return self.env.pushField(self.pos,pos) 
        return np.zeros_like(pos,dtype=np.matrix) # Not yet implemented
    
    def yawControl(self,goal):
        return self.beta*np.inner(self.navSphere(goal),skewJ*self.pose)
    
    def translatePos(self,vec):
        self.pos=self.pos+vec

    def eta(self,dist): # Dist is a scalar for distance from the obstacle
        epsilon = 1 # Dummy number
        if (epsilon**2 - dist) <= 0:
            return 0 # Avoiding div by zero glitches w/ bump function
        return np.exp(-1/(epsilon**2 - dist))/np.exp(-1/(epsilon**2)) # Epsilon is a global not yet implemented, for pushaway field width

    def alpha(self,z):
        return 1 # dummy number
    
    def beta(self,z):
        return 1 # dummy number

    def computeController(self,virtualState=None):
        # if self.task['KeepUpQ']:
        #     raise TypeError("The unicycle code does not yet support multi-agent systems.")
        
        states=self.pollNeighborsStates() # Obtain a list of neighbors' states and reduce them to positions
        if virtualState is None:
            positions={name:states[name].pos() for name in states}
        else:
            positions={}
            for name in states:
                a,b=self.network.stateVectorInfo[name]
                positions[name]=(states[name].pos(virtualState[a:b,0])).reshape(-1,1)
        
        my_pos=self.own_state(virtualState).pos()
        pnpSummand=np.zeros_like(my_pos)
        # Accumulating the network interaction component
        for name in self.neighbors: # DWR 6/25/26 Try out pnpSummand and see what happens. It's not designed to do it, so don't expect greatness.
            navvec=self.navf(positions[name],my_pos) # navvec is n from the paper
            relpos=positions[name]-my_pos # 
            navxi=self.network.tension_func(la.norm(relpos))*(la.norm(relpos)**2)/(0.+(relpos.T@navvec))
            pnpSummand=pnpSummand+navvec*navxi #reordered navvec and navxi, it gave a not aligned error since navxi is a 1x1 array
        pushvec=self.pushField(self.state.q) # This is m from the paper
        delta = 1 # dummy number
        z = 1 # dummy number
        velocity = (1-self.eta(delta**2))*self.alpha(z)*np.inner(navvec.T,(self.state.p).T) + self.eta(delta**2)*np.inner(pushvec.T,(self.state.p).T)
        angVelocity = (1-self.eta(delta**2))*self.beta(z)*np.inner(navvec.T,(skewJ*self.state.p).T) + self.eta(delta**2)*np.inner(pushvec.T,(skewJ*self.state.p).T)
        # Note: np.inner(m1,m2) prefers row matrices. Column matrices result in a square matrix output, for some reason
        return (velocity.item(),angVelocity.item()) # .item() extracts number out of a single element matrix
            # Notes
            #   veloc and angveloc come from the Unicycle PnP paper's feedback controller section on pg 1
            #   alpha, beta, and pushvec are not implemented yet.
            #   Currently the algorithm is written for a single-agent situation, as I couldn't identify much info pertaining to multi-
            #       agent situations in the Unicycle PnP paper.

    # def clientOutput(self): # May want to delete this, since there's clientoutputrobot and sim in the parent class
    #     controlInputVeloc=velocity*self.p
    #     controlInputAngle=angVelocity*skewJ*self.p
    #     return 
    
    # testing notes:
    #   Set dummy values for eta, alpha, and beta

class unicycleAgent2022(Agent):
    def __init__(self,name,env,network,task,state):
        if not(np.size(state['p'])==1):
            raise Exception("Heading should be radian")
        state=State2ndOrdRadian(np.vstack(((np.matrix(state['q'])).T,(np.matrix(state['p'])).T))) #state=self.state is done in super()
        super().__init__(name,env,network,task,state)

        rotator=np.matrix([[np.cos(state.angle),-np.sin(state.angle)],[np.sin(state.angle),np.cos(state.angle)]])
        headingVector=rotator*np.matrix([[1.],[0.]])
        self.visualization['heading']=patches.Arrow(
            x=self.state.q[0,0],
            y=self.state.q[1,0],
            dx=headingVector[0,0],
            dy=headingVector[1,0],
            alpha=1,
            color='purple',
            width=0.35,
            animated=True, # ENABLE IF DOING VIDEO
        )

        if 'safePolygon' in self.plots:
            print("add HgProj and HparaProj code in agent.py")
            # if 'HgProjection' in self.plots['safePolygon']:
            # if 'HparaProjection' in self.plots['safePolygon']:


        # DWR self notes for writing Vasilos' unicycle code:
        #   refer to pg 101 in Vasilos 2022 paper.
        #   Mapped space is polygon world after object inflation, model space is sphere world

    def dynamics(self,controlInput,inputState=None):
        if inputState is None:
            inputState=self.state
        B=np.matrix([[np.cos(inputState.angle),0.],[np.sin(inputState.angle),0.],[0.,1]])
        return B*controlInput

    def computeController(self,virtualState=None):  # See algorithm 4.2 in thesis for more info.
        # So far, being programmed for sphereworld. Will be extended to allow polygon later
        my_pos=self.own_state(virtualState).pos()
        my_heading=np.array(self.own_state(virtualState).myAngle()) # nparray for qpsolvers
        # Prepare "empty" control input vector
        controlInput=np.zeros_like(np.vstack((my_pos,np.array(my_heading)))) # not sure if vstack or hstack
        # Calculate the navigation-to-goal component
        if 'Target' in self.task:
            targ=np.matrix(np.reshape(self.network.networkInfo['networkInfo']['networkTask']['Goals'][self.task['Target']],shape=np.shape(my_pos)))

            #projected goal calc start
            # set up a qp-solve problem for the projection of the goal to the safe polygon
            goal=np.array(targ) # DWR 6/23/2026 followup: Unfortunately, qpsolvers does not like matrices.
            pos=np.array(self.state.q) # DWR 7/31/2026: Look into ways to reduce these conversion calls without affecting our intent to use npmatrix most places
            # Angular local goal setup
            HgProb=qpsolvers.problem.Problem(
                np.eye(np.size(goal)),  # minimizing squared norm
                np.zeros((np.size(goal),1)), # no linear component in this QP
                A=np.matmul((goal-pos).transpose(),skewJ), #equality constraint matrix # check to make sure the transpose works
                b=np.array([0]),  # equality constraint coefficient
                G=self.env.safetyMatrix(pos),   # DWR 7/30/2026: According to matlab code, a different safety matrix is used for unicycle
                h=self.env.safetyCoefficients(goal,pos), # safety constraints coefficients
                lb=0.5*(self.env.wkspcLowerBds+pos)-goal, # workspace boundary-safety lower bounds
                ub=0.5*(self.env.wkspcUpperBds+pos)-goal, # workspace boundary-safety upper bounds
            )
            # Linear local goal setup
            HparProb=qpsolvers.problem.Problem(
                np.eye(np.size(goal)),  # minimizing squared norm
                np.zeros((np.size(goal),1)), # no linear component in this QP
                A=np.array([-np.sin(my_heading),np.cos(my_heading)]), #equality constraint matrix, check to see if row shape works instead of column
                b=np.array([0]),  # equality constraint coefficient
                G=self.env.safetyMatrix(pos),   # safety constraints matrix
                h=self.env.safetyCoefficients(goal,pos), # safety constraints coefficients
                lb=0.5*(self.env.wkspcLowerBds+pos)-goal, # workspace boundary-safety lower bounds
                ub=0.5*(self.env.wkspcUpperBds+pos)-goal, # workspace boundary-safety upper bounds
            )
            # Standard projection to safe polygon
            safeProb=qpsolvers.problem.Problem(
                np.eye(np.size(goal)),  # minimizing squared norm
                np.zeros((np.size(goal),1)), # no linear component in this QP
                G=self.env.safetyMatrix(pos),   # safety constraints matrix
                h=self.env.safetyCoefficients(goal,pos), # safety constraints coefficients
                lb=0.5*(self.env.wkspcLowerBds+pos)-goal, # workspace boundary-safety lower bounds
                ub=0.5*(self.env.wkspcUpperBds+pos)-goal, # workspace boundary-safety upper bounds
            )
            resultHg=qpsolvers.solve_qp(P=HgProb.P,q=HgProb.q,G=HgProb.G,h=HgProb.h,A=HgProb.A,b=HgProb.b,lb=HgProb.lb,ub=HgProb.ub,solver='piqp',initvals=(pos-goal))
            resultHpar=qpsolvers.solve_qp(P=HparProb.P,q=HparProb.q,G=HparProb.G,h=HparProb.h,A=HparProb.A,b=HparProb.b,lb=HparProb.lb,ub=HparProb.ub,solver='piqp',initvals=(pos-goal))
            resultSafe=qpsolvers.solve_qp(P=safeProb.P,q=safeProb.q,G=safeProb.G,h=safeProb.h,lb=safeProb.lb,ub=safeProb.ub,solver='piqp',initvals=(pos-goal))
            if resultHg is None: resultHg=np.zeros((np.size(goal),1))
            if resultHpar is None: resultHpar=np.zeros((np.size(goal),1))
            if resultSafe is None: resultSafe=np.zeros((np.size(goal),1))
            resultHg=resultHg.reshape((np.size(goal),1))+goal
            resultHpar=resultHpar.reshape((np.size(goal),1))+goal
            resultSafe=resultSafe.reshape((np.size(goal),1))+goal
            resultHg=0.5*(resultHg+resultSafe) # Vasilos 2022 def. 55
            #projected goal calc end

            velocity=-1*np.matrix([np.cos(my_heading),np.sin(my_heading)])*(my_pos-resultHpar)
            angularV=np.arctan( (np.matrix([-1*np.sin(my_heading),np.cos(my_heading)])*(my_pos-resultHg)) / (np.matrix([np.cos(my_heading),np.sin(my_heading)])*(my_pos-resultHg)) )

        controlInput=np.vstack((velocity.transpose(),angularV),dtype=np.matrix)
        
        return controlInput