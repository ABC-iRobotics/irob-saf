clear all;

rosinit;

targetpub = rospublisher('/dvrk_vision/movement_target', 'geometry_msgs/Pose');
targetvalidpub = rospublisher('/dvrk_vision/target_valid', 'std_msgs/Bool');
targettypepub = rospublisher('/dvrk_vision/target_type', 'std_msgs/String');
statusackpub = rospublisher('/dvrk_vision/subtask_status_ack', 'std_msgs/String');
donepub = rospublisher('/dvrk_vision/task_done', 'std_msgs/Bool');
errpub = rospublisher('/dvrk_vision/error', 'std_msgs/String');

statussub = rossubscriber('/dvrk_vision/subtask_status', 'std_msgs/String');
pause(2) % Wait to ensure publisher is registered

% Start

donemsg = rosmessage(donepub);
donemsg.Data = false;
send(donepub,donemsg);

validmsg = rosmessage(targetvalidpub);
validmsg.Data = false;
send(targetvalidpub,validmsg);

status = receive(statussub);
disp(status.Data);

tgt_pos = [0.1 0.0 0.4];
tgt_ori = [0.0380 -0.0927 -0.0626 0.9930];
dp_dist = 0.04;
dp_rot = -15.0;

dist_pos = [0.1 -0.05 0.35];
dist_ori = [0.0380 -0.0927 -0.0626 0.9930];

for i = 1:3
    next_dissection = false;
    while not(next_dissection);
        disp(status.Data);
        sendStatusAck( statusackpub, status.Data )
        if strcmp(status.Data,'new_dissection_target_needed')
            % dp
            [dp_pos, dp_ori] = getDP(dp_dist, dp_rot, tgt_pos, tgt_ori );
            goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'dp', dp_pos - (i*[0.01 0.0 0.0]), dp_ori );
            
            % goal
            goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'goal', tgt_pos - (i*[0.01 0.0 0.0]), tgt_ori );
            
            next_dissection = false;
             
        elseif strcmp(status.Data, 'performing_dissection') 
            pause(0.5);
            next_dissection = false;
             
        elseif strcmp(status.Data, 'new_distant_target_needed')
          % dp
            
            [dp_pos, dp_ori] = getDP(dp_dist, dp_rot, tgt_pos, tgt_ori );
            
            goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'dp', dp_pos - (i*[0.01 0.0 0.0]), dp_ori );
            % goal
             goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'goal', dist_pos, dist_ori );
            next_dissection = true;
        
        elseif strcmp(status.Data, 'abort')
             rosshutdown;
             return;
        end
        
        status = statussub.LatestMessage;
    end
end

donemsg = rosmessage(donepub);
donemsg.Data = true;
send(donepub,donemsg);


rosshutdown;



