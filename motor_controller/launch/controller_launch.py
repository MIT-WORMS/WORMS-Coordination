from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Name of the WORM you are launching (required)'
    )
    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='false',
        description='Set to True to launch this controller with simulator gains'
    )

    # Just the PD controller node in our namespace
    nodes = [
        Node(
            package='motor_controller',
            executable='pd_controller',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'sim': LaunchConfiguration('simulator')
            }]
        )
    ]

    return LaunchDescription([
        namespace_arg,
        simulator_arg,
        *nodes
    ])