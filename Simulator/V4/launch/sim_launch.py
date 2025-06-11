#####
# For launching with simulator
#####
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
        ),
        Node(
            package='controller',

            executable='thrust_controller',
            name='thrust_translator'
        ),
        Node(
            package='controller',

            executable='position_translator',
            name='position_translator'
        ),
        Node(
            package='controller',

            executable='crane',
            name='crane'
        ),
        Node(
            package='controller',

            executable = 'rov',
            name='rov')
    ])
