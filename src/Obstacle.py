#/bin/bash python3

#obstacle class

from numpy import linalg as la
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
import random
import shapely.plotting
from shapely.geometry import Polygon
from shapely.geometry import Point

class Obstacle:
    def __init__(self,data):
        
        self.data = data
        self.num_obs = 0


    def spawn(self,data):
        self.num_obs = len(data)/4
        foo = 0
        rectangle_list = np.array([])
        for ii in range(0,self.num_obs):
            rectangle = Polygon(data[foo:foo+3])
            rectangle_list.append(rectangle)
            foo += 4


        #can buff up if needed

        return rectangle_list
    


#when obs is a shapely polygon
# 
class shapelyObstacle(Obstacle):

    def spawnPoly(data):
        return Polygon(data)
    
    def spawnSphere(center,radius):
        return Point(center).buffer(radius)
    
    def amistuck(vec,obs):
        foo = 0
        pt = Point(vec)

        for ii in range(0,len(obs)):
            o = Polygon(obs[ii])
            if o.contains(pt):
                foo += 1

        if foo>0:
            return True
        else:
            return False

    def generate_point_in_circle(center, radius,upper_,lower_):
        angle = 2 * math.pi * random.random()
        distance = radius[0][0] * math.sqrt(random.random())
        x = center[0] + distance * math.cos(angle)
        y = center[1] + distance * math.sin(angle)

        # Ensure the generated point is within the specified bounds
        x = max(min(x, upper_), lower_)
        y = max(min(y, upper_), lower_)

        return np.array([x, y])





    
    

