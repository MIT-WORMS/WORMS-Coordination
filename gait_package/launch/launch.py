from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import PopenCommand
import os
import pandas as pd

def generate_launch_description():
    command_path = "run_stand_forward_gait.csv"
    command_filepath = os.path.expanduser(f'~/WORMS-testing/src/gait_package/gait_package/{command_path}')
    df = pd.read_csv(command_filepath)
    all_worms = df.columns

    all_nodes = []

    for worm in all_worms:
        node = Node(package="gait_package",
                    namespace="gait",
                    executable="gait_manager",
                    name=worm+"_gait_manager",)
        all_nodes.append(node)
    return LaunchDescription(all_nodes)

