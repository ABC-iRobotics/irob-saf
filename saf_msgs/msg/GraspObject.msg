# GraspObject.msg
# Object info 
int8 id
string name
geometry_msgs/Point position

# Grasping parameters
geometry_msgs/Pose[] grasp_poses
geometry_msgs/Pose[] approach_poses
float64 grasp_diameter

