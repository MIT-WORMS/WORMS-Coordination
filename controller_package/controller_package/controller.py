import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class MotorController(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.subscriber = self.create_subscription(JointState, 'joint_cmd_topic', self.joint_cmd_callback, 10)
        self.subscriber = self.create_subscription(JointState, 'joint_state_topic', self.joint_state_callback, 10)
        # I want a better word rather than torque_cmd_topic
        self.publisher = self.create_publisher(JointState, 'torque_cmd_topic', 10)

    def joint_state_callback(self, msg):
        self.current_pose = msg
    def joint_cmd_callback(self, msg):
        # Update the first waypoint with the current command
        self.command = msg

        pos_error = self.current_pose.position - self.command.position

        # FIND OPTIMAL KP
        k_p = 3.0  

        # Calculate the force needed
        force = [pos_error[0] * k_p, pos_error[1] * k_p, pos_error[2] * k_p]

        # Direction of force is the sign of position error
        direction = [1 if e > 0 else -1 if e < 0 else 0 for e in pos_error]

        # Apply force direction to force vector
        force = [f * d for f, d in zip(force, direction)]

        # Publish the force as joint efforts
        joint_state_msg = JointState()
        joint_state_msg.position = self.gait_command.position
        joint_state_msg.velocity = [0.0, 0.0, 0.0] 
        joint_state_msg.effort = force

        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher = MotorController()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()