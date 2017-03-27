clear all;


rosinit;
posesub = rossubscriber('/dvrk/PSM2/position_cartesian_current', 'geometry_msgs/PoseStamped');
pause(2) % Wait to ensure publisher is registered


robot_pose_msg = posesub.LatestMessage;
    



rosshutdown;

quat = [robot_pose_msg.Pose.Orientation.W robot_pose_msg.Pose.Orientation.X robot_pose_msg.Pose.Orientation.Y robot_pose_msg.Pose.Orientation.Z];

robot_ori = quat2rotm(quat);

R = [ -0.241191,	0.879123,	0.411060;0.928426,	0.332352,	-0.166033;-0.282580	,0.341593	,-0.896361];
cam_ori = (R') * robot_ori;

cam_quat = rotm2quat(cam_ori);

disp(cam_quat);
