clear all;


rosinit;
posesub = rossubscriber('/dvrk/PSM1/position_cartesian_current', 'geometry_msgs/PoseStamped');
pause(2) % Wait to ensure publisher is registered


robot_pose_msg = posesub.LatestMessage;
    



rosshutdown;

quat = [robot_pose_msg.Pose.Orientation.W robot_pose_msg.Pose.Orientation.X robot_pose_msg.Pose.Orientation.Y robot_pose_msg.Pose.Orientation.Z];

robot_ori = quat2rotm(quat);

R = [-0.452496,	0.208165,	-0.867130; 0.883405,	0.237483,	-0.403978; 0.121835,	-0.948825,	-0.291354];

cam_ori = (R') * robot_ori;

cam_quat = rotm2quat(cam_ori);

disp(cam_quat);


 
 
 



































