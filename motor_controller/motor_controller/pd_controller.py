import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor

class PDController(Node):
    """
    A simple PD controller that takes the current joint state and the desired
    joint state as inputs, and outputs the appropriate motor effort commands. 
    """

    # Simulator gains
    SIM_KP = 3.0

    # Real world gains
    REAL_KP = 3.0

    def __init__(self) -> None:
        """
        Initializes the PD controller node with the gain passed through the rosparam.
        """
        super().__init__('pd_controller_node')
        self.declare_parameter('sim', False)

        # Controller gain
        sim = self.get_parameter('sim').get_parameter_value().bool_value
        self.kp = self.SIM_KP if sim else self.REAL_KP

        # Create subscribers (q_current and q_desired)
        self.subscriber = self.create_subscription(JointState, 'state_desired', self.joint_cmd_callback, 10)
        self.subscriber = self.create_subscription(JointState, 'state', self.joint_state_callback, 10)

        # Create the actuation publisher
        self.publisher = self.create_publisher(JointState, 'actuation_cmd', 10)

    def joint_state_callback(
            self, 
            msg: JointState
        ) -> None:
        """Updates the current state of the robot."""
        self.current_state = msg

    def joint_cmd_callback(
            self, 
            msg: JointState
        ) -> None:
        """Takes in a new desired joint state and updates the actuation to reach it."""

        # Update the first waypoint with the current command
        command = msg
        pos_error = self.current_state.position - command.position

        # Calculate the force needed
        force = [pos_error[0] * self.kp, pos_error[1] * self.kp, pos_error[2] * self.kp]

        # Direction of force is the sign of position error
        direction = [1 if e > 0 else -1 if e < 0 else 0 for e in pos_error]

        # Apply force direction to force vector
        force = [f * d for f, d in zip(force, direction)]

        # Publish the force as joint efforts
        joint_state_msg = JointState()
        joint_state_msg.effort = force
        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)

    pd_controller_node = PDController()

    executor = MultiThreadedExecutor()
    rclpy.spin(pd_controller_node, executor=executor)

    pd_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()