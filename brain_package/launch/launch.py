import os
import pandas as pd

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushROSNamespace

def generate_launch_description():
    WORKSPACE_NAME = "WORMS-software-ws"
    REPO_NAME = "WORMS-coordination"
    PACKAGE_NAME = "brain_package"

    working_file_path = os.path.dirname(os.path.realpath(__file__))
    end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
    script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)
    command_filepath = os.path.join(script_directory, "all_worms.csv")
    command_filepath = os.path.join(script_directory, "configuration_table.csv")
    df = pd.read_csv(command_filepath)
    all_worms = df["Head"]

    all_nodes = []

    for worm in all_worms:
        node = Node(package=PACKAGE_NAME,
                    executable="brain",
                    namespace=worm,
                    name='brain', # TODO: remove this line
                    remappings=[
                        (f'/{worm}/personal_communication_topic', '/personal_communication_topic'),
                        (f'/{worm}/system_communication_topic', '/system_communication_topic'),
                        (f'/{worm}/readiness_communication_topic', '/readiness_communication_topic'),
                        (f'/{worm}/state_communication_topic', '/state_communication_topic'),
                        (f'/{worm}/mission_control_command_topic', '/mission_control_command_topic')
                    ])
        all_nodes.append(node)
        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(working_file_path[:end_index], "src", REPO_NAME, 
                                                        'controller_package', 'launch', 'launch.py')])
        )
        all_nodes.append(controller_launch)

        gait_manager = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gait_manager'), 'launch', 'gait_manager_launch.py')])
        )
        gait_manager_with_namespace = GroupAction(
            actions = [
                PushROSNamespace(worm),
                gait_manager,
            ]
        )
        all_nodes.append(gait_manager_with_namespace)
    return LaunchDescription(all_nodes)