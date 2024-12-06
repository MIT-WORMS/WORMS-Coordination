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
    PACKAGE_NAME = "controller_package"

    working_file_path = os.path.dirname(os.path.realpath(__file__))
    end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
    script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)
    command_filepath = os.path.join(script_directory, "all_worms.csv")
    df = pd.read_csv(command_filepath)
    all_worms = df.columns

    all_nodes = []

    for worm in all_worms:
        controller_node = Node(package=PACKAGE_NAME,
                               executable="controller",
                               namespace=worm)
        motor_controller_node = Node(package=PACKAGE_NAME,
                                     executable='motor_controller',
                                     namespace=worm)
        all_nodes.append(controller_node)
        all_nodes.append(motor_controller_node)

    return LaunchDescription(all_nodes)