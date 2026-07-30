
import os
import sys
import Environment

import yaml
import numpy as np
from Obstacle import shapelyObstacle
from graph_w_names import graph_w_names
from Network import netwk
import matplotlib.pyplot as plt
import shapely
from time import sleep
import matplotlib.animation as animation
from scipy.integrate import odeint, solve_ivp
from matplotlib.animation import FuncAnimation, PillowWriter
from datetime import datetime
from matplotlib.lines import Line2D
plt.rcParams['animation.ffmpeg_path'] = 'C:\\0_Work_files\\programs\\ffmpeg\\bin\\ffmpeg.exe'

# from manim import *
current_time = datetime.now()
print(current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])

if len(sys.argv) != 2:
    print("Usage: main_single_sim.py <filename.yml>")
    exit()

instructions_yaml=sys.argv[1]

# Read data from the YAML file
with open(instructions_yaml, 'r') as file:
    data = yaml.safe_load(file)

simTime=data['Network']['networkInfo']['Duration']

# prepare work file names 
animationFile=data['workPath']+'pnpDemo.mp4'
solverType=data['EnvInfo']['Interface']
print(f"Solver type: {solverType}") # DWR: fixed typo Solber -> Solver
runDataFile=data['workPath']+'runData.json'

Nframes=10000 # number of frames in the animation
framesPerSec=60

print(f"World type: {data['EnvType']}")
obstacleData=data['EnvInfo']['Obstacles'] # DWR 7/8/2026: This is not needed anywhere else so far. Could be deleted
names=[]
for agentName in data['Network']['networkInfo']['Agents']['AgentInfo']:
    names.append(agentName)

def plotQuiver(target,arrowSpacing=0.7): # modify to change the fact that it was in network.py
    [xmin, ymin, xmax, ymax] = shapely.bounds(env.workspace)
    
    xArray=np.arange(xmin,xmax,arrowSpacing) # third parameter is arrow spacing. Smaller=more dense
    yArray=np.arange(ymin,ymax,arrowSpacing)
    X,Y=np.meshgrid(xArray,yArray)
    lenX=X.shape
    U=np.zeros((lenX))
    V=np.zeros((lenX))
    maxX=lenX[0]
    for idx in range(maxX):
        for idy in range(maxX):
            state=np.array([X[idx, idy], Y[idx, idy]]).reshape((2,1))
            goal=(target) # Do not convert state and goal into matrices. QPsolvers only accepts arrays
            if (env.ObsCheck(state)): # obsCheck checks if point is inside workspace minus obstacles
                navV=env.nav(goal,state)
                U[idx,idy]=navV[0,0]
                V[idx,idy]=navV[1,0]

    return X, Y, U, V
    # plt.show()

# Initialize the environment
env=getattr(Environment,data['EnvType'])(data['EnvInfo'])

# Initialize the visualization
figure,visualization=plt.subplots()
visualDict={}
visualDict['environmentPlot']=visualization
# network visualization must happen after environment is visualized

# Environment.py plotting. obsBuffer and collarPolygon are mainly for testing
visualization.add_patch(env.workspacePatch())
#visualization.add_patch(env.obstacleBufferPlot())
#visualization.add_patch(env.collarPolygonPlot())

# Quiver plotting
target=np.array((data['Network']['networkInfo']['networkTask']['Goals']['Goal1'])).reshape((2,1))
qX,qY,qU,qV=plotQuiver(target,arrowSpacing=0.7)
visualization.quiver(qX,qY,qU,qV)

# Initialize the network
net=netwk(data['Network'],env,visualDict)
stateVector=net.formNetworkStateVector() # for later use in solving ODEs

###DAN'S BS HERE
###This needs to be done in Network.py, maybe "populate"?
#for agentName:info in data['Agents']['AgentInfo']: # 6/10/26 Dan code
#    self.agents[name]=getattr(agent,info['Type'])(**info['State'])
### NECESSARY ADJUSTMENTS:
# need the Agent class (and subclasses) to have a way of responding to an absent state input (has to do with the randomized spawning algorithm Ishan implemented)

# This section needs to be edited to reflect new yml structure
pnpParameters=data['Network']['networkInfo']['pnpParameters']
# each edge is represented as a tuple of names
if len(names) > 1:
    edges=[tuple(item) for item in data['Network']['networkInfo']['Edges']]
else:
    edges=None

graph=graph_w_names(names,edges)

# Network.py plotting (some may be moved into their own categories, like stuff that comes from graph_w_names.py)
if net.LazyQ:
    titlePlot='Lazy PnP Controller'
else:
    titlePlot='Contractive PnP Controller'

# misc. plotting
visualization.grid(False)
# Trajectory plotting is handled by plot_multi_agent_trajectories


def updateAni(content): # Content is assumed to be a tuple containing timestamp,networkState,net
    timeStamp,networkState,net=content
    # update agent state
    net.pnpUpdate(networkState)
    # one tick of the network clock
    net.tick(timeStamp)
    # update the visualization data
    net.updateVisualization() # needs to be updated in network.py to reflect changes to how visualization works

def plot_multi_agent_trajectories(net, odeSol, flowTime):
    num_agents = len(net.graph.names)
    
    ax = visualization
    colors = plt.cm.rainbow(np.linspace(0, 1, num_agents))
    goal_x, goal_y = 9,9
    ax.plot(goal_x, goal_y, 'X', color='red', markersize=15, markeredgewidth=2, label='Goal')
    # Plot communication graph edges
    for name in net.graph.names:
        agent_index = net.graph.vertexIndices[name]
        a,b=net.stateVectorInfo[name]
        agentPos=odeSol[0,a:a+2]
        neighbors = net.neighbors(name)
        for neighbor in neighbors:
            a,b=net.stateVectorInfo[neighbor]
            neighborPos=odeSol[0,a:a+2] # DWR 7/2/2026: Can't remember if we have a pre-existing system for telling which part of odeSol is the position rather than heading
            ax.plot([agentPos[0], neighborPos[0]], [agentPos[1], neighborPos[1]], 'k-', linewidth=1.5)  # Black lines for initial configuration
    # Plot trajectories
    for i, name in enumerate(net.graph.names):
        agent_index = net.graph.vertexIndices[name]
        a,b=net.stateVectorInfo[name]
        agentPos=odeSol[:,a:a+2] # DWR 7/2/2026: Can't remember if we have a pre-existing system for telling which part of odeSol is the position rather than heading
        ax.plot(agentPos[:,0], agentPos[:,1], '--', color=colors[i], label=f'Agent {name}', linewidth=0.95)  # Reduced line thickness
        ax.plot(agentPos[0,0], agentPos[0,1], 'o', color=colors[i], markersize=8)  # Start point
        ax.plot(agentPos[-1,0], agentPos[-1,1], 'o', color=colors[i], markersize=8)  # End point

    # Plot initial configuration with black edges between neighbors
    for name in net.graph.names:
        agent_index = net.graph.vertexIndices[name]
        x = odeSol[-1, 2*agent_index]
        y = odeSol[-1, 2*agent_index + 1]
        neighbors = net.neighbors(name)
        for neighbor in neighbors:
            neighbor_index = net.graph.vertexIndices[neighbor]
            nx = odeSol[-1, 2*neighbor_index]
            ny = odeSol[-1, 2*neighbor_index + 1]
            ax.plot([x, nx], [y, ny], 'k-', linewidth=1.5)

    # Ensure aspect ratio is equal
    ax.set_aspect('equal', adjustable='box')

    # Create custom legend
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', label='Agents', markerfacecolor='gray', markersize=8),
        Line2D([0], [0], color='k', label='Communication Graph'),
        # Line2D([0], [0], marker='o', color='w', label='Obstacles', markerfacecolor='red', markersize=8),
        Line2D([0], [0], marker='>', color='gray', label='Navigation Field'),
        Line2D([0], [0], marker='X', color='red', markersize=8, markeredgewidth=1, label='Goal'),
        Line2D([0], [0], linestyle='--', color='gray', label='Agent Trajectory')  
    ]
    figure.text(0.5,0.01,'MAS Simulation with Graph Maintainance',ha='center',fontsize=11)
    # Add legend to lower right corner
    ax.legend(handles=legend_elements, loc='lower right',  ncol=1,fontsize=18)

def frameCull(timeData,stateData,maxTime,desiredNframes):
    mask=np.zeros(len(timeData),dtype=bool)
    timeBtwFrames=maxTime/desiredNframes
    frameNum=-1 #how many frames were "made" so far
    for step in range(len(timeData)):
        maskValue=np.floor(timeData[step]/timeBtwFrames)>frameNum
        if maskValue:
            mask[step]=True
            frameNum+=1
    return timeData[mask],stateData[mask]

def frameCounter(timeData,stateData,netObject): # 7/1/2026: input in frames is supposed to be a generator, not a list
    for timestamp,networkState in zip(timeData,stateData): 
        yield timestamp,networkState,netObject

if solverType=='Euler':
    flowTime=np.linspace(0,simTime,simTime*framesPerSec)

    for timestep in flowTime:
        net.pnpUpdate()
        net.updateVisualization()

    ani=animation.FuncAnimation(
        fig=net.figure,
        func=updateAni,
        # frames=frameCounter(Nframes,net),
        frames=[net for item in range(Nframes)], 
        interval=1,
        # cache_frame_data=False,
        save_count=Nframes,
    )
    myPath=os.path.abspath(__file__)
    # animationFile = r"/home/ishan/sims/variable_graph_MAS/sims/" 
    writerVideo = animation.FFMpegWriter(fps=framesPerSec) 
    ani.save('pnpMovieContractive.mp4', writer=writerVideo)

elif solverType == 'nsfPlots':

    flowTime = np.linspace(0, simTime, simTime*framesPerSec)
    odeSol,output_dict=odeint(net.FlowMap,stateVector.T.flatten(),flowTime,full_output=1)
    print(odeSol, output_dict)

    # Call the plotting function
    plot_multi_agent_trajectories(net, odeSol, flowTime)
    plt.tight_layout()
    plt.title('ODE Solution for Multi-Agent Contractive PnP Controller')
    plt.show()

elif solverType=="odeInt": #For OdeInt
    flowTime=np.linspace(0,simTime,simTime*framesPerSec)
    # plt.show()
    # raise Exception("Comment this line out to turn on the ode solver")
    print(stateVector)
    odeSol,output_dict=odeint(net.FlowMap,stateVector.T.flatten(),flowTime,full_output=1)
    # odeObj=solve_ivp(net.FlowMapSwapInput,[0,simTime],stateVector.flatten(),method='RK45')
    # odeSol=odeObj['y'].transpose() # solve_ivp has y as columns left to right, whereas our code wants y as rows up to down
    print(f"odeSol: {odeSol}")
    print(f"odeSol shape: {np.shape(odeSol)}")
    plt.plot(odeSol[:,0],odeSol[:,1],'b--')
    plot_multi_agent_trajectories(net, odeSol, flowTime)
    plt.title('ODE Solution for Multiple Agents')
    plt.show()
    raise Exception("Comment this line out to turn on the video")
    odeTimeStamps=np.insert(output_dict['tcur'],0,0) # odeSol includes the starting t=0 frame, but t=0 is not included in tcur
    odeTimeStamps,odeSol=frameCull(odeTimeStamps,odeSol,maxTime=simTime,desiredNframes=Nframes)
    ani=animation.FuncAnimation(
        fig=net.figure,
        func=updateAni,
        frames=frameCounter(odeTimeStamps,odeSol,net),
        # frames=[net for item in range(Nframes)], 
        interval=1,
        # cache_frame_data=False,
        save_count=Nframes,
    )
    myPath=os.path.abspath(__file__)
    net.plotEdgeLenghts()
    writerVideo = animation.FFMpegWriter(fps=framesPerSec) 
    ani.save('pnpMovie.mp4', writer=writerVideo)
