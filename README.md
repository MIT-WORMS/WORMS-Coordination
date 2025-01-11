# WORMS-coordination
Welcome to the WORMS-coordination repository!


WORMS-coordination is the repository that houses all of the individual worm-specific nodes. 

The structures of the specific packages and nodes are shown below.
- brain_package
  - action_node
  - brain
- controller_package
  - controller
  - motor_controller

The following topics will be used:
Brain:
- action_topic - used to interface the action_node with the readiness checks conducted by the brain
- joint_states -
- coordination_topic - used to send completion-level messages (either "in_progress" or "done")
- joint_commands -
- readiness_communication_topic -
- personal_communication_topic -
- system_communication_topic - 

Coontroller_node:
- joint_cmd_topic - used to send the position commands for the worm
- torque_cmd_topic - used to send torque commands to the motors
- joint_state_topic - used to send the current state of the worm (used for any sort of feedback control)


A more in-depth description of each package, node, and topic is given below.

brain_package - Responsible for initiating each worm. Includes a personal and system-level readiness check and handling of commands to be sent to a gait manager.

controller_package - Responsible for handling joint_commands as determined by the gait manager or manipulation manager (not a part of this repo). Includes a simple PD controller.

action_node - 

brain - Responsible for handling communication between worms. Records and relays the state of the worm and publishes commands downstream. TODO: handle the configuration of the system in a smarter way.

controller - Responsible for receiving joint position commands and producing torque commands.

motor_controller - Responsible for sending torque commands to motors and reading resulting motor positions.
