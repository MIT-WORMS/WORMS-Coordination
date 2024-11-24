from launch import LaunchDescription
from launch_ros.actions import Node
import os
import pandas as pd
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    WORKSPACE_NAME = "WORMS-software-ws"
    REPO_NAME = "WORMS-coordination"
    PACKAGE_NAME = "brain_package"

    working_file_path = os.path.dirname(os.path.realpath(__file__))
    end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
    script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)
    command_filepath = os.path.join(script_directory, "all_worms.csv")
    df = pd.read_csv(command_filepath)
    all_worms = df.columns

    all_nodes = []

    for worm in all_worms:
        node = Node(package=PACKAGE_NAME,
                    executable="brain",
                    namespace=worm,
                    name='brain', # TODO: remove this line
                    remappings=[
                        (f'/{worm}/personal_communication_topic', '/personal_communication_topic'),
                        (f'/{worm}/system_communication_topic', '/system_communication_topic'),
                        (f'/{worm}/readiness_communication_topic', '/readiness_communication_topic')
                    ])
        all_nodes.append(node)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controller_package'), 'launch', 'launch.py')])
    )
    all_nodes.append(controller_launch)
    return LaunchDescription(all_nodes)