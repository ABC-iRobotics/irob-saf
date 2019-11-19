clear all;


rosinit;
posesub = rossubscriber('/dvrk/PSM1/position_cartesian_current', 'geometry_msgs/PoseStamped');
pause(2) % Wait to ensure publisher is registered


robot_pose_msg = posesub.LatestMessage;
    



rosshutdown;

quat = [robot_pose_msg.Pose.Orientation.W robot_pose_msg.Pose.Orientation.X robot_pose_msg.Pose.Orientation.Y robot_pose_msg.Pose.Orientation.Z];

robot_ori = quat2rotm(quat);

R = [-0.085659	,-0.987034	,-0.135744;-0.568080,	0.160314,	-0.807208;0.818503	,0.007969,	-0.574446];
cam_ori = (R') * robot_ori;

cam_quat = rotm2quat(cam_ori);

disp(cam_quat);
