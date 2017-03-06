function [ targetpub, targetvalidpub, targettypepub, statusackpub, donepub, errpub, statussub ] = dvrkInit()


targetpub = rospublisher('/dvrk_vision/movement_target', 'geometry_msgs/Pose');
targetvalidpub = rospublisher('/dvrk_vision/target_valid', 'std_msgs/Bool');
targettypepub = rospublisher('/dvrk_vision/target_type', 'std_msgs/String');
statusackpub = rospublisher('/dvrk_vision/subtask_status_ack', 'std_msgs/String');
donepub = rospublisher('/dvrk_vision/task_done', 'std_msgs/Bool');
errpub = rospublisher('/dvrk_vision/error', 'std_msgs/String');

statussub = rossubscriber('/dvrk_vision/subtask_status', 'std_msgs/String');
pause(2) % Wait to ensure publisher is registered

% Start

sendDone( donepub, false );

validmsg = rosmessage(targetvalidpub);
validmsg.Data = false;
send(targetvalidpub,validmsg);

status = receive(statussub);
disp(status.Data);


end

