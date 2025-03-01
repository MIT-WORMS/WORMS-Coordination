import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import pandas as pd
import subprocess
import platform
import os

    
class CommandPublisher(Node):

    def __init__(self, spreadsheet_path):
        """
        Creates node 'action_node'.

        Publishes to all worm_action_topic and subscribes to all worm_coordination_topic.

        Creates timer that publishes next in sequence of commands if all worms are done moving.
        """

        super().__init__("action_node")
        df = pd.read_csv(spreadsheet_path)
        self.all_worms = df.columns

        self.publisher_dict = {}
        for i in self.all_worms:
            self.publisher_dict[i] = self.create_publisher(String, f'{i}_action_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Command Publisher node initialized")

        self.action_callbacks = {i: self.create_action_callbacks(i) for i in self.all_worms}
        self.subscribers = {i: self.create_subscription(String, f'{i}_coordination_topic', self.action_callbacks[i], 10) 
                            for i in self.all_worms}
        
        self.status = {i : 1 for i in self.all_worms}

        self.command_list = {i: df[i] for i in self.all_worms}
        self.current_index = 0

    def publish_command(self, worm_id):
        """
        Publishes next command to appropriate topic and logs data.
        """
        if self.current_index < len(self.command_list[worm_id]):
            data = self.command_list[worm_id][self.current_index]
            msg = String()
            msg.data = data
            self.publisher_dict[worm_id].publish(msg)
            self.get_logger().info(f"Published command {msg.data} to {worm_id}")
            self.current_index += 1

    def create_action_callbacks(self, worm_id):
        """
        Creates action_callback function for a given worm_id
        """
        def action_callback(msg):
            """
            Tells Node whether worm is done moving.
            """
            if msg.data == "done":
                self.status[worm_id] = 1
            else:
                self.status[worm_id] = 0
        
        return action_callback
    
    def timer_callback(self):
        """
        Publishes next command if all worms are done moving or if running first command.
        """
        all_done = True
        for i in self.status:
            if not i:
                all_done = False
        
        if all_done or self.current_index == 0:
            for worm in self.all_worms:
                self.publish_command(worm)
        
        else:
            print("WAITING FOR ALL WORMS TO COMPLETE ACTIONN")

def main(args=None):
    """
    Initializes ROS, creates 'action_node' Node, and creates command_list
    """
    rclpy.init(args=args)

    # i.e. run_stand_forward_gait.csv
    command_path = input("Enter a command filename: ")
    # command_path = "run_stand_forward_gait.csv"
    # command_filepath = os.path.expanduser(f'~/worms_mech_ws/src/worms_mech/worms_mech/{command_path}')
    command_filepath = os.path.expanduser(f'~/WORMS-testing/src/gait_package/gait_package/{command_path}')

    command_publisher = CommandPublisher(command_filepath)

    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        pass
    command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()