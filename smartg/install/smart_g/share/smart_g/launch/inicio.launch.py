import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import time

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_g',
            executable='VideoInput',
            name='video_input',
            output='screen',
        ),
        # Add a delay of 5 seconds before starting the next node
        launch.actions.TimerAction(
            period = 10.0,
            actions=[
                launch.actions.LogInfo(
                    msg="Waiting for 10 seconds before starting the next node...",
                ),
            ]
        ),
        Node(
            package='smart_g',
            executable='leaf_define',
            name='leaf_define',
            output='screen',
        ),
        launch.actions.TimerAction(
            period = 10.0,
            actions=[
                launch.actions.LogInfo(
                    msg="Waiting for 10 seconds before starting the next node...",
                ),
            ]
        ),
        Node(
            package='smart_g',
            executable='detector',
            name='detector',
            output='screen',
        ),
        launch.actions.TimerAction(
            period = 10.0,
            actions=[
                launch.actions.LogInfo(
                    msg="Waiting for 10 seconds before starting the next node...",
                ),
            ]
        ),
        Node(
            package='smart_g',
            executable='tracker',
            name='obj_Tracker',
            output='screen',
        ),
        launch.actions.TimerAction(
            period = 10.0,
            actions=[
                launch.actions.LogInfo(
                    msg="Waiting for 10 seconds before starting the next node...",
                ),
            ]
        ),
        Node(
            package='smart_g',
            executable='outputNode',
            name='output_node',
            output='screen',
        ),
    ])
