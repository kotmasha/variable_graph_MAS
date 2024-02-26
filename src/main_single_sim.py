
import os
import sys
from Environment import environment
from Environment import sphereworldEnv
from Environment import polygonEnv
from Environment import starWorldEnv
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

if len(sys.argv) != 3:
    print("Usage: main_single_sim.py <filename.yml>")
    exit()

instructions_yaml=sys.argv[1]
simTime=int(sys.argv[2])
# Read data from the YAML file
with open(instructions_yaml, 'r') as file:
    data = yaml.safe_load(file)

# prepare work file names 
animationFile=data['workPath']+'pnpMovie.mp4'
runDataFile=data['workPath']+'runData.json'
Nframes=10000 # number of frames in the animation
worldType=data['Obstacles']['worldType']
obstacleData=data['Obstacles']
upper=int(data['Wksp_bnds']['upper'][0][0])
lower=int(data['Wksp_bnds']['lower'][0][0])
names=data['Agents']['Names']
agentTypes={}
if len(data['Agents'])>4:
    stateWname=data['Agents']['NamesandPos']
else:
    stateWname=None
for name, obj in globals().items():
    if isinstance(obj, type):  # Check if the object is a class
        if name in data['Agents']['AgentType']:
            agentTypes[name] = obj
pnpParameters=data['pnpParameters']
# each edge is represented as a tuple of names
edges=[tuple(item) for item in data['Agents']['tree_mat_names']]
leaders=data['Agents']['Leaders']
netID=data['netID']
wksp_coords=((lower, lower), (lower, upper), (upper, upper), (upper, lower), (lower, lower))
outerbounds=shapely.Polygon(wksp_coords)
# env=environment(outerbounds,obstacleData)
if worldType==0:    
    env=sphereworldEnv(outerbounds,obstacleData)
elif worldType==1:
    env=starWorldEnv()    
elif worldType==2:
    env=polygonEnv()

graph=graph_w_names(names,edges)
net=netwk(netID,graph,env,leaders,pnpParameters,stateWname)

# step=0.001
# for timestep in range(0,Nframes,step):
#     net.pnpUpdate()
#     net.updateVisualization()
# sys.exit()
def updateAni(content):
    # update agent positions
    # content.dummyUpdate()
    content.pnpUpdate()
    # update the visualization data
    content.updateVisualization()

def frameCounter(n,obj):
    for frame in range(n):
        yield obj
flowTime=np.linspace(0,simTime,simTime*60)
odeSol=odeint(net.pnpFlowMap,net.y0.flatten(),flowTime,full_output=1,mxstep=50)



plt.legend(loc='best')


# ani=animation.FuncAnimation(
#     fig=net.figure,
#     func=updateAni,
#     # frames=frameCounter(Nframes,net),
#     frames=[net for item in range(Nframes)], 
#     interval=1,
#     # cache_frame_data=False,
#     save_count=Nframes,
# )
# animationFile = r"c://Users/ishan/Desktop/animation.mp4" 
# writerVideo = animation.FFMpegWriter(fps=60) 
# ani.save('pnpDemo.mp4', writer=writerVideo)
plt.show()    




