clear all;


rosinit;
posesub = rossubscriber('/dvrk/PSM2/position_cartesian_current', 'geometry_msgs/PoseStamped');
pause(2) % Wait to ensure publisher is registered


robot_pose_msg = posesub.LatestMessage;
    



rosshutdown;

quat = [robot_pose_msg.Pose.Orientation.W robot_pose_msg.Pose.Orientation.X robot_pose_msg.Pose.Orientation.Y robot_pose_msg.Pose.Orientation.Z];

robot_ori = quat2rotm(quat);

R = [  -0.6352 ,   0.7508  ,  0.1810;   -0.0898 ,   0.1610 ,  -0.9829;   -0.7671 ,  -0.6406 ,  -0.0349];

cam_ori = (R') * robot_ori;

cam_quat = rotm2quat(cam_ori);

disp(cam_quat);


 
 
 



































