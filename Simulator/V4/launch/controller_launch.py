 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            #namespace='controller',
            executable='usv',
            name='usv'
        ),
        Node(
            package='controller',
            #namespace='controller',
            executable='alloc',
            name='allocator'
        )
    ])
