# This represents the force-torque given to an object relative to its current state.
Header header

geometry_msgs/Vector3 force
geometry_msgs/Vector3 torque

float32 force_duration
float32 torque_duration


# int command type present the type of message given (which of the parameters above to use.
# 1 for force
# 2 torque

int32 command_types