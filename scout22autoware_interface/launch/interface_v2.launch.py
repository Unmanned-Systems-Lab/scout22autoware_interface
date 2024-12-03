from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scout22autoware_interface',
            executable='AUTO2SCOUT',
            name='Autoscoutpub'
        ),
        Node(
            package='scout22autoware_interface',
            executable='SCOUT2AUTO',
            name='Autoscoutsub'
        ),
    ])