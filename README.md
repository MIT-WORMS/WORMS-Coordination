# WORMS-coordination
Welcome to the WORMS-coordination repository!


WORMS-coordination is the repository that houses all of the individual worm-specific nodes. 

The structures of the specific packages and nodes are shown below.
- brain
  - action_node
  - brain
- motor_controller
  - pd_controller
  - motor_interface

The following topics will be used:

Brain:
- action_topic - used to interface the action_node with the readiness checks conducted by the brain
- joint_states -
- coordination_topic - used to send completion-level messages (either "in_progress" or "done")
- joint_commands -
- readiness_communication_topic -
- personal_communication_topic -
- system_communication_topic - 

Motor Controller:
- state_desired - used to send the position commands for the worm
- actuation_cmd - used to send torque commands to the motors
- state - used to send the current state of the worm (used for any sort of feedback control)



Packages and Nodes:

brain - Responsible for initiating each worm. Includes a personal and system-level readiness check and handling of commands to be sent to a gait manager.

motor_controller - Responsible for handling joint_commands as determined by the gait manager or manipulation manager (not a part of this repo). Includes a simple PD controller.

action_node - 

brain - Responsible for handling communication between worms. Records and relays the state of the worm and publishes commands downstream. TODO: handle the configuration of the system in a smarter way.

pd_controller - Responsible for receiving joint position commands and producing torque commands.

motor_interface - Responsible for sending torque commands to motors and reading resulting motor positions.
