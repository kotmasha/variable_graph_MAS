import os
import sys
from Environment import environment
from Environment import sphereworldEnv
from Environment import polygonEnv
from Environment import starworldEnv
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

# from manim import *
current_time = datetime.now()
print(current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])

if len(sys.argv) != 3:
    print("Usage: main_single_sim.py <filename.yml> <simulation time in seconds>")
    exit()

instructions_yaml=sys.argv[1]
simTime=int(sys.argv[2])
# Read data from the YAML file
with open(instructions_yaml, 'r') as file:
    data = yaml.safe_load(file)

# prepare work file names 
animationFile=data['workPath']+'pnpDemo.mp4'
solverType=data['solverType']
print(f"Solber type: {solverType}")
runDataFile=data['workPath']+'runData.json'
Nframes=10000 # number of frames in the animation
worldType=data['Obstacles']['worldType']
print(f"World type: {worldType}")
obstacleData=data['Obstacles']
upper=int(data['Wksp_bnds']['upper'][0][0])
lower=int(data['Wksp_bnds']['lower'][0][0])
names=data['Agents']['Names']
agentSpawn=data['agentSpawn']
edgesQ=data['Agents']['edgesQ']
ramdomQ=data['Agents']['randomQ']
stateWnameQ=int(data['Agents']['stateWnames'])
agentTypes={}
if stateWnameQ==1:
    stateWname=data['Agents']['NamesandPos']    
elif stateWnameQ==2:
    stateWname=data['Agents']['NamesandPosandPose']    
else:
    stateWname=None

pnpParameters=data['pnpParameters']
# each edge is represented as a tuple of names
if edgesQ:
    edges=[tuple(item) for item in data['Agents']['tree_mat_names']]
else:
    edges=None
leaders=data['Agents']['Leaders']
netID=data['netID']
wksp_coords=((lower, lower), (lower, upper), (upper, upper), (upper, lower), (lower, lower))
outerbounds=shapely.Polygon(wksp_coords)
# env=environment(outerbounds,obstacleData)
if worldType==0:    
    env=sphereworldEnv(outerbounds,obstacleData)
elif worldType==1:
    env=starworldEnv(outerbounds,obstacleData)    
elif worldType==2:
    env=polygonEnv(outerbounds,obstacleData)

graph=graph_w_names(names,edges)
net=netwk(netID,graph,env,leaders,pnpParameters,agentSpawn,simTime,worldType,stateWname,ramdomQ)
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