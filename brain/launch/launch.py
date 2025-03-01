import os
import pandas as pd
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    REPO_NAME = "WORMS-Coordination"
    PACKAGE_NAME = "brain"

    # ../ros2_ws/install/share/foo -> back out ros2_ws
    ws_directory = Path(get_package_share_directory('brain')).parent.parent.parent.parent
    script_directory = ws_directory / "src" / REPO_NAME / PACKAGE_NAME / PACKAGE_NAME
    command_filepath = script_directory / "all_worms.csv"
    command_filepath = script_directory / "configuration_table.csv"
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
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('motor_controller'), 'launch', 'motor_launch.py')]),
                launch_arguments={'namespace': worm}.items()
        )
        all_nodes.append(controller_launch)

        # gait_manager = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('gait_manager'), 'launch', 'gait_manager_launch.py')])
        # )
        # gait_manager_with_namespace = GroupAction(
        #     actions = [
        #         PushRosNamespace(worm),
        #         gait_manager,
        #     ]
        # )
        # all_nodes.append(gait_manager_with_namespace)
    return LaunchDescription(all_nodes)