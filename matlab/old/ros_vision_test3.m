%   @author: Renata Elek

clear all;
close all;
%rosinit;



%[ I_l, I_r ] = stereo_capture( 1, 'BGR24_640x480' );
I_r = imread('img_l_1.jpg');
I_l = imread('img_r_1.jpg');
cuttingXYZ = XYZ_coordinate_calculation( I_l, I_r, 'calibrationSession.mat', 'stereoParams.mat', ...
    400, 0, 20, 20 );

tgt_pos = cuttingXYZ(1, :);
tgt_ori = [0.0380 -0.0927 -0.0626 0.9930];
dp_dist = 0.04;
dp_rot = -15.0;


dist_pos = [0.0279287680855560,0.00,0.28]

dist_ori = [0.0380 -0.0927 -0.0626 0.9930];

[targetpub, targetvalidpub, targettypepub, statusackpub, donepub, errpub, statussub] ...
        = dvrkInit();

status = statussub.LatestMessage;

for i = 1:3
    tgt_pos = cuttingXYZ(20*(i-1)+1, :);
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

sendDone(donepub, true);

rosshutdown;