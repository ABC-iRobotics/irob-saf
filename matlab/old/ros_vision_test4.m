%   @author: Renata Elek

clear all;
close all;
rosinit;



[ I_r, I_l ] = stereo_capture( 1, 'BGR24_640x480' );
%I_l = imread('img_l_1.jpg');
%I_r = imread('img_r_1.jpg');
cuttingXYZ = XYZ_coordinate_calculation( I_l, I_r, 'calibrationSession.mat', 'stereoParams.mat', ...
    400, 0, 8, 8 );

tgt_pos = cuttingXYZ(1, :);
tgt_ori = [ 0.9825   , 0.1830 ,   0.0179 ,  -0.0294];
dp_dist = 0.04;
dp_rot = -15.0;


dist_pos = [0.0133340582644101,-0.0600216146603506,0.307688160427651];
dist_ori = [ 0.9825  ,  0.1830 ,   0.0179 ,  -0.0294];

[targetpub, targetvalidpub, targettypepub, statusackpub, donepub, errpub, statussub] ...
        = dvrkInit();

status = statussub.LatestMessage;
gotodistant = false;
stepPix = 20;
s = size(cuttingXYZ, 1);
n = (s / stepPix)+1;
for i = 1:n
    tgt_pos = cuttingXYZ(stepPix*(i-1)+1, :);
    tgt_pos = tgt_pos - [0.0, 0.0, 0.0];
    next_dissection = false;
    while not(next_dissection);
        disp(status.Data);
        sendStatusAck( statusackpub, status.Data )
        gotodistant = false;
        if  i == n
            gotodistant = false;
        end
        if strcmp(status.Data,'new_dissection_target_needed')
            % dp
            [dp_pos, dp_ori] = getDP(dp_dist, dp_rot, tgt_pos, tgt_ori );
            goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'dp', dp_pos, dp_ori );
            
            % goal
            goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'goal', tgt_pos, tgt_ori );
            
            next_dissection = false;
             
        elseif strcmp(status.Data, 'performing_dissection') 
            pause(0.5);
            next_dissection = false;
             
        elseif strcmp(status.Data, 'new_distant_target_needed')
            if gotodistant
                % dp
                [dp_pos, dp_ori] = getDP(dp_dist, dp_rot, tgt_pos, tgt_ori );
            
                goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'dp', dp_pos, dp_ori );
                % goal
                goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'goal', dist_pos, dist_ori );
            else
                 % goal
                [dp_pos, dp_ori] = getDP(dp_dist, dp_rot, tgt_pos, tgt_ori );
            
                goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, ...
                                'goal', dp_pos, dp_ori );
            end
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