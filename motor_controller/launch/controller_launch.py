from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Name of the WORM you are launching (required)'
    )
    namespace = LaunchConfiguration('namespace')

    # Just the PD controller node in our namespace
    nodes = [
        Node(
            package='motor_controller',
            executable='pd_controller',
            namespace=namespace
        )
    ]

    return LaunchDescription([
        namespace_arg,
        *nodes
    ])