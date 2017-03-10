clear all;


rosinit;
posesub = rossubscriber('/dvrk/PSM2/position_cartesian_current', 'geometry_msgs/PoseStamped');
pause(2) % Wait to ensure publisher is registered


robot_pose_msg = posesub.LatestMessage;
    



rosshutdown;

quat = [robot_pose_msg.Pose.Orientation.W robot_pose_msg.Pose.Orientation.X robot_pose_msg.Pose.Orientation.Y robot_pose_msg.Pose.Orientation.Z];

robot_ori = quat2rotm(quat);

R = [ -0.314386,	0.883408,	-0.347494;0.948909	,0.302893	,-0.088480;0.027090	,-0.357557	,-0.933498];

cam_ori = (R') * robot_ori;

cam_quat = rotm2quat(cam_ori);

disp(cam_quat);


 
 
 



































