import sys
import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='matplotlib_sim',
            executable='matplotlib_sim_node',
            name='matplotlib_sim',
        ),
        Node(
            package='single_integrator_agent',
            namespace='r1',
            executable='single_int_node',
            name='single_integrator_agent_1',
            parameters=[
                {'x_init': 3.0},
                {'y_init': 2.0},
                {'name': 'r1'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r2',
            executable='single_int_node',
            name='single_integrator_agent_2',
            parameters=[
                {'x_init': -1.0},
                {'y_init': -1.0},
                {'name': 'r2'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r3',
            executable='single_int_node',
            name='single_integrator_agent_3',
            parameters=[
                {'x_init': 5.0},
                {'y_init': -1.0},
                {'name': 'r3'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r4',
            executable='single_int_node',
            name='single_integrator_agent_4',
            parameters=[
                {'x_init': -5.0},
                {'y_init': 1.0},
                {'name': 'r4'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r5',
            executable='single_int_node',
            name='single_integrator_agent_5',
            parameters=[
                {'x_init': 1.0},
                {'y_init': -2.5},
                {'name': 'r5'},
            ],
        ),
    ])
