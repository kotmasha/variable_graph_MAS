
import os
import sys
import Environment
import numpy as np
import yaml
from Obstacle import shapelyObstacle
from graph_w_names import graph_w_names
from Network import netwk
import matplotlib.pyplot as plt
import shapely
from time import sleep
import matplotlib.animation as animation
from scipy.integrate import odeint
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
worldType=data['EnvType']
print(f"World type: {worldType}")
obstacleData=data['EnvInfo']['Obstacles']
names=[]
for agentName in data['Network']['networkInfo']['Agents']['AgentInfo']:
    names.append(agentName)
# stateWnameQ=int(data['Agents']['stateWnames']) # DWR 6/15/2026: this block should probably be removed, since we want to detect if starting positions are given based on the yml file
# agentTypes={}
# if stateWnameQ==1:
#     stateWname=data['Agents']['AgentInfo']  
# else:
#     stateWname=None

# Initialize the environment
env=getattr(Environment,data['EnvType'])(data['EnvInfo']) # DWR 6/15/26 the ** on **data[] is not necessary in this situation

# Initialize the network
net=netwk(data['Network'],env)
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
# net=netwk(netID,graph,env,leaders,pnpParameters,agentSpawn,simTime,worldType,stateWname) #This line, and possibly others nearby, should be deleted since the network is being started up earlier


def updateAni(content):
    # update agent positions
    # content.dummyUpdate()
    content.pnpUpdate()
    # update the visualization data
    content.updateVisualization()

def plot_multi_agent_trajectories(net, odeSol, flowTime):
    num_agents = len(net.graph.names)
    
    ax = net.visualization
    colors = plt.cm.rainbow(np.linspace(0, 1, num_agents))
    goal_x, goal_y = 9,9
    ax.plot(goal_x, goal_y, 'X', color='red', markersize=15, markeredgewidth=2, label='Goal')
    for name in net.graph.names:
        agent_index = net.graph.vertexIndices[name]
        x = odeSol[0, 2*agent_index]
        y = odeSol[0, 2*agent_index + 1]
        neighbors = net.neighbors(name)
        for neighbor in neighbors:
            neighbor_index = net.graph.vertexIndices[neighbor]
            nx = odeSol[0, 2*neighbor_index]
            ny = odeSol[0, 2*neighbor_index + 1]
            ax.plot([x, nx], [y, ny], 'k-', linewidth=1.5)  # Black lines for initial configuration
    # Plot trajectories
    for i, name in enumerate(net.graph.names):
        agent_index = net.graph.vertexIndices[name]
        x = odeSol[:, 2*agent_index]
        y = odeSol[:, 2*agent_index + 1]
        ax.plot(x, y, '--', color=colors[i], label=f'Agent {name}', linewidth=0.95)  # Reduced line thickness
        ax.plot(x[0], y[0], 'o', color=colors[i], markersize=8)  # Start point
        ax.plot(x[-1], y[-1], 'o', color=colors[i], markersize=8)  # End point

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
    net.figure.text(0.5,0.01,'MAS Simulation with Graph Maintainance',ha='center',fontsize=11)
    # Add legend to lower right corner
    ax.legend(handles=legend_elements, loc='lower right',  ncol=1,fontsize=18)
if solverType=='Euler':
    flowTime=np.linspace(0,simTime,simTime*60)

    # for timestep in flowTime:
    #     net.pnpUpdate()
    #     net.updateVisualization()

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
    writerVideo = animation.FFMpegWriter(fps=60) 
    ani.save('pnpMovieContractive.mp4', writer=writerVideo)

elif solverType == 'nsfPlots':

    flowTime = np.linspace(0, simTime, simTime*60)
    odeSol,output_dict=odeint(net.FlowMap,stateVector.T.flatten(),flowTime,full_output=1)
    print(odeSol, output_dict)

    # Call the plotting function
    plot_multi_agent_trajectories(net, odeSol, flowTime)
    plt.tight_layout()
    plt.title('ODE Solution for Multi-Agent Contractive PnP Controller')
    plt.show()

elif solverType=="odeSolo": # Needs to be fixed or removed
    flowTime=np.linspace(0,simTime,simTime*60)
    odeSol,output_dict=odeint(net.pnpFlowMapsolo,net.agents['Zoe'].pos.flatten(),flowTime,full_output=1)
    # print(odeSol)
    plt.plot(odeSol[:,0],odeSol[:,1],'b--')
    plt.title('ODE Solution for Single Agent')
    plt.show()

elif solverType=="odeInt": #For OdeInt
    flowTime=np.linspace(0,simTime,simTime*60)
    print(f"stateVector:{stateVector}")
    odeSol,output_dict=odeint(net.FlowMap,stateVector.T.flatten(),flowTime,full_output=1)
    print(f"odeSol:{odeSol}")
    # plt.plot(odeSol[:,0],odeSol[:,1],'b--')
    plot_multi_agent_trajectories(net, odeSol, flowTime)
    plt.title('ODE Solution for Multiple Agents')
    plt.show()
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
    net.plotEdgeLenghts()

    raise Exception("The animation currently takes a long time to run. Comment this out if you have some time to kill.")
    # animationFile = r"/home/ishan/sims/variable_graph_MAS/sims/" 
    writerVideo = animation.FFMpegWriter(fps=60) 
    ani.save('pnpMovie.mp4', writer=writerVideo)




def frameCounter(n,obj):
    for frame in range(n):
        yield obj






