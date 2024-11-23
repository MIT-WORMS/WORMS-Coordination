import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class MotorSet(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        self.subscriber = self.create_subscription(JointState, 'torque_cmd_topic', self.torque_cmd_callback, 10)
        self.publisher = self.create_publisher(JointState, 'joint_state_topic', 10)
        
        # Hardcoded to match pican board
        can_device = 'can0'
        motor_ids = [1, 2, 3] # figure out how to make this general

        self.pos1 = 0
        self.pos2 = 0
        self.pos3 = 0

        for motor_id in motor_ids:
            self.motor_controller_dict[motor_id] = CanMotorController(can_device, motor_id, motor_type="AK80_6_V2")

        # Figure out motor initialization? PICAN stuff?

    
    def torque_cmd_callback(self, msg):
        # Handle the incoming joint state command messages here
        joint_state_msg = JointState()

        # Update motor states dynamically
        for idx, (motor_id, motor_controller) in enumerate(self.motor_controller_dict.items()):
            # Ensure message length matches motor count
            if idx < len(msg.position):
                Kp = 45
                Kd = 2
                pos_command = msg.position[idx]
                vel_command = msg.velocity[idx]
                K_ff = msg.effort[idx]

                # Send command and update state
                pos, vel, curr = motor_controller.send_deg_command(pos_command, vel_command, Kp, Kd, K_ff)
                self.motor_states[motor_id]["position"] = pos
                self.motor_states[motor_id]["velocity"] = vel
                self.motor_states[motor_id]["effort"] = curr

        # Publish updated joint states
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"motor_{motor_id}" for motor_id in self.motor_ids]
        joint_state_msg.position = [self.motor_states[motor_id]["position"] for motor_id in self.motor_ids]
        joint_state_msg.velocity = [self.motor_states[motor_id]["velocity"] for motor_id in self.motor_ids]
        joint_state_msg.effort = [self.motor_states[motor_id]["effort"] for motor_id in self.motor_ids]


    def set_zero_position(self, motor):
        motor.set_zero_position()

    def on_shutdown(self):
        self.get_logger().info("Disabling Motors...")
        for motor_id, motor_controller in self.motor_controller_dict.items():
            motor_controller.disable_motor()


def main(args=None):
    rclpy.init(args=args)

    motor_controller_node = MotorSet()

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass

    motor_controller_node.on_shutdown()
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()