import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # publisher to send joint commands
        self.cmd_publisher = self.create_publisher(JointState, '/input_joint_cmds', 10)
        # publisher to send joint states (for PD feedback error)
        self.state_publisher = self.create_publisher(JointState, '/input_joint_state', 10)

        # subsribe to "torques" which is really just a JointState message\
        self.subscription = self.create_subscription(
            JointState, 'torque_cmd_topic', self.listener_callback, 10)
        self.received_message  = None

    def listener_callback(self, msg):
        self.received_message = msg
    

