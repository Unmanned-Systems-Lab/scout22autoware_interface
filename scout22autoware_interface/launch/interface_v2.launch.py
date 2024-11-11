from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interface_v2',
            executable='AUTO2SCOUT',
            name='Autoscoutpub'
        ),
        Node(
            package='interface_v2',
            executable='SCOUT2AUTO',
            name='Autoscoutsub'
        ),
    ])