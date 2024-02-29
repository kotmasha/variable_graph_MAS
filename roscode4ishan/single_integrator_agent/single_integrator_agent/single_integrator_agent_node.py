#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os
import math

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import quaternion_from_euler

#Takes in twist, publishes position & heading

#Define your class - use package name with camel case capitalization
class SingleIntegrator(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('single_integrator_node')

        #Define parameters here
        self.declare_parameter('x_init', 0.0)
        self.declare_parameter('y_init', 0.0)
        self.declare_parameter('name', 'r1')

        #Get parameters here
        x = self.get_parameter('x_init').value
        y = self.get_parameter('y_init').value
        self.name = self.get_parameter('name').value
        self.pos = [x,y]

        #Define the publishers here
        #self.'publisher_var_name' = self.create_publisher('MessageType', '/topic_name', 'queue length')
        self.pose_pub_ = self.create_publisher(PoseStamped, 'pose', 10)
        self.global_pose_pub_ = self.create_publisher(PoseStamped, '/pose', 10)

        #Define the subscribers here
        #self.'subscriber_var_name' = self.create_subscription('MessageType', '/topic_name', self.'callback_function', 'queue length')
        self.twist_sub_ = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        #Variable to track the current time
        self.current_time = self.get_clock().now()
        self.cmd_epoch = self.get_clock().now()
        #Variable to store data from callback messages that can be used globally
        self.twist = Twist()

        #Set the timer period and define the timer function that loops at the desired rate
        self.time_period = 1/10
        self.timer = self.create_timer(self.time_period, self.timer_callback)


    #This is the timer function that runs at the desired rate from above
    def timer_callback(self):
        # #update position based on twist msg
        # self.pos[0] = self.pos[0] + self.time_period*self.twist.twist.linear.x*math.cos(self.twist.twist.angular.z)
        # self.pos[1] = self.pos[1] + self.time_period*self.twist.twist.linear.x*math.sin(self.twist.twist.angular.z) + self.time_period*self.twist.twist.linear.x

        if (self.get_clock().now()-self.cmd_epoch).nanoseconds*(10**-9) > 1:
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0

        self.pos[0] = self.pos[0] + self.time_period*self.twist.linear.x
        self.pos[1] = self.pos[1] + self.time_period*self.twist.linear.y


        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.name
        msg.pose.position.x = self.pos[0]
        msg.pose.position.y = self.pos[1]
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0,0,self.twist.angular.z)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pose_pub_.publish(msg)
        self.global_pose_pub_.publish(msg)

        #This is how to keep track of the current time in a ROS2 node
        self.current_time = self.get_clock().now()

    #Put your callback functions here - these are run whenever the node
    #loops and when there are messages available to process.
    def twist_callback(self, msg):
        self.twist = msg
        self.cmd_epoch = self.get_clock().now()

#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    single_integrator = SingleIntegrator()
    rclpy.spin(single_integrator)
    single_integrator.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
