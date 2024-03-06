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
                {'x_init': -14.0},
                {'y_init': -14.0},
                {'name': 'r1'},
                {'neighbors': ['r2']},
            ],
        ),
        Node(
            package='pnp_control',
            namespace='r1',
            executable='pnp_control_node',
            name='pnp_control_node_1',
            parameters=[
                {'max_vel': 0.25},
                {'K': 1.0},
                {'name': 'r1'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r2',
            executable='single_int_node',
            name='single_integrator_agent_2',
            parameters=[
                {'x_init': -14.0},
                {'y_init': -13.0},
                {'name': 'r2'},
                {'neighbors': ['r1','r3']},
            ],
        ),
        Node(
            package='pnp_control',
            namespace='r2',
            executable='pnp_control_node',
            name='pnp_control_node_2',
            parameters=[
                {'max_vel': 0.25},
                {'K': 1.0},
                {'name': 'r2'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r3',
            executable='pnp_control_node',
            name='pnp_control_node_3',
            parameters=[
                {'x_init': -13.0},
                {'y_init': -12.0},
                {'name': 'r3'},
                {'neighbors': ['r2','r4','r5']},
            ],
        ),
        Node(
            package='pnp_control',
            namespace='r3',
            executable='consensus_control_node',
            name='consensus_control_node_3',
            parameters=[
                {'max_vel': 0.25},
                {'K': 1.0},
                {'name': 'r3'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r4',
            executable='single_int_node',
            name='single_integrator_agent_4',
            parameters=[
                {'x_init': -12.0},
                {'y_init': -13.0},
                {'name': 'r4'},
                {'neighbors': ['r3']},
            ],
        ),
        Node(
            package='pnp_control',
            namespace='r4',
            executable='pnp_control_node',
            name='pnp_control_node_4',
            parameters=[
                {'max_vel': 0.25},
                {'K': 1.0},
                {'name': 'r4'},
            ],
        ),
        Node(
            package='single_integrator_agent',
            namespace='r5',
            executable='single_int_node',
            name='single_integrator_agent_5',
            parameters=[
                {'x_init': -14.0},
                {'y_init': -10.0},
                {'name': 'r5'},
                {'neighbors': ['r3']},
            ],
        ),
        Node(
            package='pnp_control',
            namespace='r5',
            executable='pnp_control_node',
            name='pnp_control_node_5',
            parameters=[
                {'max_vel': 0.25},
                {'K': 1.0},
                {'name': 'r5'},
            ],
        ),
    ])
