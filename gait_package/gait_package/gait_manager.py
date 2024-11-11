import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import pandas as pd
import subprocess
import platform
import os
import time
import sys

from messages.msg import Personal
from messages.msg import System
from messages.msg import Readiness

def get_mac_address():
    """Returns MAC address of wlan0 network interface."""
    mac_address = subprocess.check_output(f"cat /sys/class/net/wlan0/address", shell=True).decode().strip()

    if mac_address:
        return mac_address
        
    print("Error getting MAC address: No suitable interface found")
    return None

def find_robot_name(mac_address, spreadsheet_path):
    """
    Returns species name based on mac_address.
    
    Args:
    - mac_address: str, MAC address, retreived from get_mac_address
    - spreadsheet_path: str, file path to spreadsheet database.csv
    """
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        return None
    
def find_location(head, spreadsheet_path):
    """
    Returns location based on head.

    This should be abstracted to some other way of finding location.

    Args:
    - head: str, head
    - spreadsheet_path: file path to spreadsheet configuration_table.csv
    """
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['Head']==head]
    if not match.empty:
        return match.iloc[0].iloc[1]
    else:
        return None

def find_configuration(head, spreadsheet_path):
    """
    Returns species specialiation (motor directions) based on head.txt.

    This should be abstracted to some other way of finding configuration.

    Args:
    - head: str, head, read from head.txt
    - spreadsheet_path: file path to spreadsheet configuration_table.csv
    """
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['Head'] == head]
    if not match.empty:
        # this should be tweaked if the format of specialization_table.csv changes
        # also worm should be able to figure out where it is on the robot, don't have that feature yet
        return list(match.iloc[0].iloc[2:].values)
    else:
        return None

class JointCommandPublisher(Node):

    def __init__(self):
        """
        Creates 'gait_manager' Node.

        Communication between worms:
        Publishes and subscribes to 'personal_communication_topic', 'system_communication_topic',
        and 'readiness_communication_topic'. All worms use these topics to communicate with each other.

        Each worm is a state machine.
        - "Idle": the worm is not ready: worms do not agree on entire system configuration or config
            is not valid
        - "Ready": the worm is ready: worm believes every other worm has the same view of the system
            and that system is valid
        - "Active": the worm is active and requesting/publishing movement commands: all worms are
            constantly communicating that they are ready

        Communication with low level controllers:
        Publishes to 'joint_commands' and 'coordination_topic' topics.
        Subscribes to 'joint_states' (joint_state_callback), 'action_topic'
        (action_callback). Each worm publishes to topics that move JointState commands downstream.
        """
        super().__init__("blank")

        WORKSPACE_NAME = "WORMS-software-ws"
        REPO_NAME = "WORMS-coordination"
        PACKAGE_NAME = "gait_package"

        working_file_path = os.path.dirname(os.path.realpath(__file__))
        end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
        self.script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)

        self.configuration_path = os.path.join(self.script_directory, "configuration_table.csv")

        self.worm_id = self.get_name().split("_")[0]

        self.location = find_location(self.worm_id, self.configuration_path)
        self.configuration = find_configuration(self.worm_id, self.configuration_path)

        """
        These variables define the direction of the motors that are specific to the current gait assembled
        In the case of our 4 legged system, we use 1 and -1 to define which side each worm would be on in the 
        standard forward operating condition, which makes:
        ------------------------
        1 = Left Side Leg
        -1 = Right Side Leg
        ------------------------
        The Effect of the -1 on the Right side is to flip the direction of the head motor when executing 
        the same step command
        """

        joint_commands_topic = f'{self.worm_id}_joint_commands'
        joint_states_topic = f'{self.worm_id}_joint_states'
        coordination_topic = f'{self.worm_id}_coordination_topic'
        action_topic = f'{self.worm_id}_action_topic'

        self.command_publisher = self.create_publisher(JointState, joint_commands_topic, 10)
        self.coordination_publisher = self.create_publisher(String, coordination_topic, 10)
        self.state_subscriber = self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)
        self.action_subscriber = self.create_subscription(String, action_topic, self.action_callback, 10)

        self.personal_publisher = self.create_publisher(Personal, 'personal_communication_topic', 10)
        self.system_publisher = self.create_publisher(System, 'system_communication_topic', 10)
        self.personal_subscriber = self.create_subscription(Personal, 'personal_communication_topic', self.personal_communication_callback, 10)
        self.system_subscriber = self.create_subscription(System, 'system_communication_topic', self.system_communication_callback, 10)

        self.readiness_publisher = self.create_publisher(Readiness, 'readiness_communication_topic', 10)
        self.readiness_subscriber = self.create_subscription(Readiness, 'readiness_communication_topic', self.readiness_communication_callback, 10)

        self.state = "Idle"
        self.system_view = System()
        self.system_view.sender = self.worm_id
        self.all_system_views = {}

        self.readiness = {}
        self.message_delay = {}
        self.message_delay_threshold = 13 # change this to not be hard coded

        self.num_motors = len(self.configuration)
        self.position_index = 0
        self.current_position = [0 for _ in range(self.num_motors)]

        self.execute_timer_callback = False
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.config_timer = self.create_timer(0.5, self.config_callback)
        
    def get_waypoints(self, action):
        """
        Returns waypoints from csv file given some action.
        """
        waypoints_path = os.path.join(self.script_directory, "gait_data", f"{action}.csv")

        df = pd.read_csv(waypoints_path)
        return [list(i) for i in df.values]
    
    def set_personal_view(self, name, location, configuration):
        """
        Generates view of self using Personal custom data type.
        
        Args:
        - name: (str) own name, typically self.worm_id
        - location: (str) location on robot i.e. RightLeftLeg
        - configuration: (list of ints) multipliers for joints
        """
        self.personal_view = Personal()
        self.personal_view.name = name
        self.personal_view.location = location
        self.personal_view.configuration = configuration
    
    def personal_communication_callback(self, msg):
        """
        Receives Personal message and updates view of system.

        Args:
        - msg (Personal): message
        """
        if msg.name != self.worm_id:
            not_seen_yet = True
            for i in range(len(self.system_view.system_config)):
                if self.system_view.system_config[i].name == msg.name:
                    self.system_view.system_config[i] = msg
                    not_seen_yet = False
                    break
            if not_seen_yet:
                self.system_view.system_config.append(msg)


    def system_communication_callback(self, msg):
        """
        Receives System message and updates view of system.

        Args:
        - msg (System): message
        """
        self.all_system_views[msg.sender] = msg
        
    def readiness_communication_callback(self, msg):
        """
        Receives message and updates self.readiness and self.message_delay

        Args:
        - msg (Readiness): message
        """
        if msg.sender != self.worm_id:
            self.readiness[msg.sender] = msg.status in ["Ready", "Active"]

            for i in self.readiness:
                if msg.sender == i:
                    self.message_delay[i] = 0
                else:
                    if i in self.message_delay:
                        self.message_delay[i] += 1
                    else:
                        self.message_delay[i] = 1


    def all_systems_valid(self):
        """
        First checks if all systems present are equal. Then checks if system is valid.
        """
        for system in self.all_system_views.values():
            # comparing each system in all_system_views to self.system_view
            for personal in system.system_config:
                # there must be one and only copy
                copy_exists = False
                one_copy = False
                for p in self.system_view.system_config:
                    if p.name == personal.name and p.location == personal.location and p.configuration == personal.configuration:
                        if not one_copy:
                            copy_exists = True
                            one_copy = True
                        else:
                            copy_exists = False
                if not copy_exists:
                    return False

        # this should be changed to something else
        all_worms = set()
        all_locations = set()
        for personal in self.system_view.system_config:
            all_worms.add(personal.name)
            all_locations.add(personal.location)
        if len(all_worms) == len(all_locations) == 6:
            return True
        return False

    def joint_state_callback(self, msg):
        """
        Updates self.current_pose and self.current_position based on received message. Logs new position.

        Args:
        - msg: message received from f'/{worm_id}_joint_states' topic
        """
        
        print("Updating Joint State")
        # Update the first waypoint with the current position
        self.current_pose = msg

        if hasattr(self, 'current_pose') and self.current_pose is not None:
            position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
            self.get_logger().info(f'Current Position at: [{position_str}]')
            self.current_position = self.current_pose.position

    def interpolate_waypoints(self, arrays, increment=0.3):
        """
        Transition from the current position to each array in sequence by 0.1 increments, producing a comprehensive list of
        lists representing each incremental step towards the waypoints.

        :param arrays: A sequence of arrays where each array is a list of numbers.
        :param increment: The incremental value to adjust the numbers. Default is 0.1.
        :return: A list of lists representing each step through the waypoints.
        """
        transition_steps = []

        # Start from the current position
        current_state = self.current_position

        for target_array in arrays:
            while True:
                step = []
                done = True
                for current_val, target_val in zip(current_state, target_array):
                    if abs(target_val - current_val) > increment:
                        done = False
                        if target_val > current_val:
                            step.append(current_val + increment)
                        else:
                            step.append(current_val - increment)
                    else:
                        step.append(target_val)
                
                current_state = step
                transition_steps.append(step)
                
                if done:
                    break

        return transition_steps

    def config_callback(self):
        self.configuration = find_configuration(self.worm_id, self.configuration_path)
        self.location = find_location(self.worm_id, self.configuration_path)

        self.set_personal_view(self.worm_id, self.location, self.configuration)

        # update personal view in system_view
        in_system_view = False
        for i in range(len(self.system_view.system_config)):
            if self.system_view.system_config[i].name == self.worm_id:
                self.system_view.system_config[i] = self.personal_view
                in_system_view = True
        if not in_system_view:
            self.system_view.system_config.append(self.personal_view)

        # update system_view in all_system_views
        self.all_system_views[self.worm_id] = self.system_view

        self.personal_publisher.publish(self.personal_view)
        self.system_publisher.publish(self.system_view)

        
        
        if not self.all_systems_valid():
            self.state = "Idle"
        elif self.state == "Idle":
            self.state = "Ready"

        msg = Readiness()
        msg.sender = self.worm_id

        msg.status = self.state

        self.readiness_publisher.publish(msg)

        # logic to check for if a worm has died
        all_worms_ready = True
        for worm in self.readiness:
            if not self.readiness[worm]:
                all_worms_ready = False
        for worm, value in self.message_delay.items():
            if value >= self.message_delay_threshold:
                for i, personal in enumerate(self.system_view.system_config):
                    if personal.name == worm:
                        del self.system_view.system_config[i]
                        break
                all_worms_ready = False
        if set([i for i in self.readiness.keys()]) != (set([i for i in self.all_system_views.keys()])-set([self.worm_id])):
            all_worms_ready = False

        if all_worms_ready:
            self.state = "Active"
        # if self.state is Active but not all worms are ready, move down to Ready
        elif self.state == "Active":
            self.state = "Ready"
        
        self.get_logger().info(f"{self.state}")
    
    def timer_callback(self):
        """
        Publishes position command based on next entry in interpolated positions.

        Publishes JointState message with desired position to relevant joint_commands topic. 
        When finished with command, sets execute_timer_callback to False and publishes final JointState
        message to joint_commands topic.

        Publishes either 'in_progress' or 'done' to coordination_topic each time.
        """
        
        msg = String()
        msg.data = "static"

        if self.state == "Active":
            if self.execute_timer_callback:
                msg.data = "in_progress"

                if self.position_index < len(self.interpolated_positions):
                    self.position_command = list(map(float, self.interpolated_positions[self.position_index]))

                    self.get_logger().info(f"Publishing command: {self.position_command}")

                    self.position_command = [self.position_command[i]*self.configuration[i] for i in range(self.num_motors)]

                    self.position_index += 1

                else:
                    msg.data = "done"

                    self.position_index = 0
                    self.execute_timer_callback = False

                    if hasattr(self, 'current_pose') and self.current_pose is not None:
                        position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
                        self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')


                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.position = self.position_command
                joint_state_msg.velocity = [0.0 for _ in range(self.num_motors)]
                joint_state_msg.effort = [0.0 for _ in range(self.num_motors)]
                self.command_publisher.publish(joint_state_msg)

            self.coordination_publisher.publish(msg)

    def action_callback(self, msg):
        """
        Defines self.interpolated_positions and self.action. Sets self.execute_timer_callback to True

        Args:
        - msg: message from action_topic
        """

        waypoints = self.get_waypoints(msg.data)
        self.interpolated_positions = self.interpolate_waypoints(waypoints)
        self.action = msg.data
        self.execute_timer_callback = True

def main(args=None):
    """
    Initializes ROS, creates 'gait_manager' node.
    """
    rclpy.init(args=args)

    # command_path = "run_stand_forward_gait.csv"
    # command_filepath = os.path.expanduser(f'~/WORMS-testing/src/gait_package/gait_package/{command_path}')
    # df = pd.read_csv(command_filepath)
    # all_worms = df.columns

    # executor = MultiThreadedExecutor()

    # for worm in all_worms:
    #     executor.add_node(JointCommandPublisher(worm))

    # # executor.set_number_of_threads(8)

    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass
    
    # executor.shutdown()
    # for i in executor.get_nodes():
    #     i.destroy_node()
    # executor.shutdown()
    # rclpy.shutdown()

    # rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

            



        