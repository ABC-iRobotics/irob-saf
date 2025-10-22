# Environment.msg

int8 VALID = 1
int8 INVALID = 2

# Msg data
std_msgs/Header header
int8 valid

geometry_msgs/Transform tf_phantom

GraspObject[] objects
