# Material types
int8 RIGID=1
int8 SOFT=2

# Msg data
Header header

int8 id
string name
geometry_msgs/Pose pose
geometry_msgs/Pose grasp_pose
float64 grasp_diameter
int8 material
