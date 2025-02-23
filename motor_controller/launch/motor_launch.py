from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Name of the WORM you are launching (required)'
    )
    namespace = LaunchConfiguration('namespace')

    # Launch the controller with the selected namespace
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motor_controller'),
                'launch',
                'controller_launch.py'
            )
        ),
        launch_arguments={'namespace': namespace}.items()
    )

    # Also launch the motor interface for the controller
    nodes = [
        Node(
            package='motor_controller',
            executable='motor_interface',
            namespace=namespace
        )
    ]

    return LaunchDescription([
        namespace_arg,
        *nodes,
        controller_launch
    ])