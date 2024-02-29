#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os
import math
import matplotlib.pyplot as plt

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from geometry_msgs.msg import PoseStamped
from dataclasses import dataclass

@dataclass
class agent_:
    name: str
    x: float
    y: float
    x_hist: list[float]
    y_hist: list[float]
    z: float
    yaw: float


#Define your class - use package name with camel case capitalization
class MatPlotLibSim(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('matplotlib_sim_node')

        #Define the subscribers here
        self.pose_sub_ = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)

        #Variable to store data from callback messages that can be used globally
        #it uses that Header message type since that's the message type on the topic
        self.agents = []

        plt.ion()

        self.BBox = [-10, 10, -10, 10]
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.agent_plot, = self.ax.plot([],[],'ro', label="agent")
        self.agent_hist_plot, = self.ax.plot([],[],'k,', label="agent path")
        self.ax.set_xlabel('Easting')
        self.ax.set_ylabel('Northing')
        self.ax.legend()
        self.ax.grid()
        self.ax.axis(xmin=self.BBox[0],xmax=self.BBox[1], ymin=self.BBox[2],ymax=self.BBox[3])
        self.ax.axis('equal')


        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.timer = self.create_timer(time_period, self.timer_callback)


    #This is the timer function that runs at the desired rate from above
    def timer_callback(self):

        #rclpy.spin_once(self,timeout_sec=1/100)
        t_x = []
        t_y = []
        t_hist_x = []
        t_hist_y = []
        if len(self.agents)>0:
            for ii in range(len(self.agents)):
                #self.target_plot = self.axis1.plot(self.targets[ii].x,self.targets[ii].y,'ro', label="targets")
                t_x.append(self.agents[ii].x)
                t_y.append(self.agents[ii].y)
                for jj in range(len(self.agents[ii].x_hist)):
                    t_hist_x.append(self.agents[ii].x_hist[jj])
                    t_hist_y.append(self.agents[ii].y_hist[jj])

        self.agent_plot.set_xdata(t_x)
        self.agent_plot.set_ydata(t_y)
        self.agent_hist_plot.set_xdata(t_hist_x)
        self.agent_hist_plot.set_ydata(t_hist_y)

        self.fig.canvas.draw()

        self.fig.canvas.flush_events()

    #Put your callback functions here - these are run whenever the node
    #loops and when there are messages available to process.
    def pose_callback(self, msg):
        if len(self.agents) < 1: #first agent found on topic, add its state
            agent = agent_(msg.header.frame_id, msg.pose.position.x, msg.pose.position.y,  [msg.pose.position.x], [msg.pose.position.y], msg.pose.position.z, 0)
            self.agents.append(agent)
        elif len(self.agents) >= 1: #more than one agent found on topic
            it = 0
            for ii in range(len(self.agents)): #loop through list to see if its already been seen on topic
                if msg.header.frame_id == self.agents[ii].name: #if true we've see its state before
                    it = it+1
                    #agent = agent_(msg.header.frame_id, msg.pose.position.x, msg.pose.position.y,  [msg.pose.position.x], [msg.pose.position.y], msg.pose.position.z, 0) #make new struct
                    #self.agents[ii] = agent #update its state
                    self.agents[ii].x = msg.pose.position.x
                    self.agents[ii].y = msg.pose.position.y
                    self.agents[ii].x_hist.append(msg.pose.position.x)
                    self.agents[ii].y_hist.append(msg.pose.position.y)
            if it == 0: #that means we didn't see new agent in the list
                agent = agent_(msg.header.frame_id, msg.pose.position.x, msg.pose.position.y, [msg.pose.position.x], [msg.pose.position.y], msg.pose.position.z, 0) #make new struct for this new agent
                self.agents.append(agent) #add it to the list

#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    matplotlib_sim = MatPlotLibSim()
    rclpy.spin(matplotlib_sim)
    matplotlib_sim.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()

#if running in python3 use
#if __name__ == 'main':
#    main()
