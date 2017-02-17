clear all;

rosinit;

targetpub = rospublisher('/dvrk_vision/movement_target', 'geometry_msgs/Pose');
targetvalidpub = rospublisher('/dvrk_vision/target_valid', 'std_msgs/Bool');
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

for i = 1:3
    next_dissection = false;
    while not(next_dissection)
        disp(status.Data);
        if strcmp(status.Data,'new_dissection_target_needed')
            targetmsg = rosmessage(targetpub);
            targetmsg.Position.X =  -0.0352589864065 + ((i-1) * 0.02);
            targetmsg.Position.Y = -0.0622652795347;
            targetmsg.Position.Z = -0.0603390324918;
            targetmsg.Orientation.X = 0.572063821814;
            targetmsg.Orientation.Y = 0.703609906823;
            targetmsg.Orientation.Z = -0.415625078075;
            targetmsg.Orientation.W = 0.0702273256427;
            send(targetpub,targetmsg);
            
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = true;
            send(targetvalidpub,validmsg);
            
            status = statussub.LatestMessage;
            
            while not(strcmp(status.Data,'target_reached'))
                send(targetpub,targetmsg);
                pause(0.5);
                status = statussub.LatestMessage;
            end
            
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = false;
            send(targetvalidpub,validmsg);
            
            pause(0.5);
            next_dissection = false;
             
        elseif strcmp(status.Data, 'performing_dissection') 
            pause(0.5);
            next_dissection = false;
             
        elseif strcmp(status.Data, 'new_distant_target_needed')
            targetmsg = rosmessage(targetpub);
            targetmsg.Position.X =  0.0;
            targetmsg.Position.Y = 0.0;
            targetmsg.Position.Z =  -0.0534999999991;
            
            targetmsg.Orientation.X = 0.707106781172;
            targetmsg.Orientation.Y = 0.707106781191;
            targetmsg.Orientation.Z = 0.0;
            targetmsg.Orientation.W = -0.0000259734823723;
            send(targetpub,targetmsg);
            
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = true;
            send(targetvalidpub,validmsg);
            
            status = statussub.LatestMessage;
            
            while not(strcmp(status.Data, 'target_reached'))
                send(targetpub,targetmsg);
                pause(0.5);
                status = statussub.LatestMessage;
            end
                         
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = false;
            send(targetvalidpub,validmsg);
            
            pause(0.5);
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


% pose: 
%   position: 
%     x: -0.0352589864065
%     y: -0.0622652795347
%     z: -0.0603390324918
%   orientation: 
%     x: 0.572063821814
%     y: 0.703609906823
%     z: -0.415625078075
%     w: 0.0702273256427






% pose: 
%   position: 
%     x: 1.96516473024e-07
%     y: 2.29942639466e-07
%     z: -0.0534999999991
%   orientation: 
%     x: 0.707106781172
%     y: 0.707106781191
%     z: 2.59734823723e-06
%     w: -2.59734823723e-06



























