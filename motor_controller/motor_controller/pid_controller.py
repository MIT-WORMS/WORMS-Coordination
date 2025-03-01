import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor

class PIDController(Node):
    """
    A simple PID controller that takes the current joint state and the desired
    joint state as inputs, and outputs the appropriate motor effort commands. 
    """

    # Simulator gains
    SIM_KP = 150.0
    SIM_KD = 30.0
    SIM_KI = 0.0

    # Real world gains
    REAL_KP = 3.0
    REAL_KD = 0.0
    REAL_KI = 0.0

    def __init__(self) -> None:
        """
        Initializes the PID controller node with the gain passed through the rosparam.
        """
        super().__init__('pid_controller_node')
        self.declare_parameter('sim', False)
        self.command = None

        # Controller gain
        sim = self.get_parameter('sim').get_parameter_value().bool_value
        self.kp = self.SIM_KP if sim else self.REAL_KP
        self.kd = self.SIM_KD if sim else self.REAL_KD
        self.ki = self.SIM_KI if sim else self.REAL_KI

        # Integral accumulation
        self.int_error = np.zeros(3)

        # Create subscribers (q_current and q_desired)
        self.subscriber = self.create_subscription(JointState, 'state_desired', self.joint_cmd_callback, 10)
        self.subscriber = self.create_subscription(JointState, 'state', self.joint_state_callback, 10)

        # Create the actuation publisher
        self.publisher = self.create_publisher(JointState, 'actuation_cmd', 10)

    def joint_state_callback(
            self, 
            msg: JointState
        ) -> None:
        """Takes in a new state of the robot and updates the actuation to reach it."""
        if not self.command: return

        # Update the first waypoint with the current command
        current_state = msg
        pos_error = np.array(current_state.position) - np.array(self.command.position)
        vel_error = np.array(current_state.velocity)

        # Calculate the force needed
        force = pos_error * -self.kp + vel_error * -self.kd + self.int_error * -self.ki

        # Publish the force as joint efforts
        joint_state_msg = JointState()
        joint_state_msg.effort = force.tolist()
        self.publisher.publish(joint_state_msg)

        # Accumulate integral
        self.int_error += pos_error

    def joint_cmd_callback(
            self, 
            msg: JointState
        ) -> None:
        """Updates the current command of the robot."""
        self.command = msg

def main(args=None):
    rclpy.init(args=args)

    pid_controller_node = PIDController()

    executor = MultiThreadedExecutor()
    rclpy.spin(pid_controller_node, executor=executor)

    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()