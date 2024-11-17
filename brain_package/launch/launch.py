from launch import LaunchDescription
from launch_ros.actions import Node
import os
import pandas as pd

def generate_launch_description():
    WORKSPACE_NAME = "WORMS-software-ws"
    REPO_NAME = "WORMS-coordination"
    PACKAGE_NAME = "brain_package"

    working_file_path = os.path.dirname(os.path.realpath(__file__))
    end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
    script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)
    command_filepath = os.path.join(script_directory, "run_stand_forward_gait.csv")
    df = pd.read_csv(command_filepath)
    all_worms = df.columns

    all_nodes = []

    for worm in all_worms:
        node = Node(package=PACKAGE_NAME,
                    executable="brain",
                    namespace=worm,
                    name='brain',
                    remappings=[
                        (f'/{worm}/personal_communication_topic', '/personal_communication_topic'),
                        (f'/{worm}/system_communication_topic', '/system_communication_topic'),
                        (f'/{worm}/readiness_communication_topic', '/readiness_communication_topic')
                    ])
        all_nodes.append(node)
    return LaunchDescription(all_nodes)