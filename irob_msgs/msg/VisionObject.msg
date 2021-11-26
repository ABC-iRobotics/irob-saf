# VisionObject.msg
# Material types
int8 RIGID=1
int8 SOFT=2


# Object info 
int8 id
string name
geometry_msgs/Transform transform

# Grasping parameters
geometry_msgs/Transform grasp_transform
float64 grasp_diameter
int8 material

