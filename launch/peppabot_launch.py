from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for PeppaBot path planning
        Node(
            package='peppabot',
            executable='peppabot_node',
            name='peppabot',
            output='screen',
            parameters=[{
                # Add any parameters you may want to configure at runtime, e.g.,
                # "goal_topic": "/goal_pose"
            }]
        ),
        
        # # Optionally, include other nodes like the navigation stack
        # Node(
        #     package='nav2_bringup',
        #     executable='bringup_launch',
        #     output='screen',
        #     # If needed, specify launch arguments or params
        # )
    ])

