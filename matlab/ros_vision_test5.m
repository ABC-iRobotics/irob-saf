%   @author: Renata Elek

clear all;
close all;
rosinit;

groupN = 30;



tgt_ori = [  0.9938    0.0705    0.0514    0.0685];
dp_dist = 0.04;
dp_rot = 345.0;


dist_pos = [0.0133340582644101,-0.0000216146603506,0.20];
dist_ori = [ 0.9825  ,  0.1830 ,   0.0179 ,  -0.0294];

[targetpub, targetvalidpub, targettypepub, statusackpub, donepub, errpub, statussub] ...
    = dvrkInit();

status = statussub.LatestMessage;
gotodistant = false;
stepPix = 10;
lastIdx = 0;
firstTgt = true;
userInputX = [0, 0];
userInputY = [0, 0];

%s = size(cuttingXYZ, 1);
n = (groupN / stepPix)+1;
for j = 1:3
    
    [ I_r, I_l ] = stereo_capture( 1, 'BGR24_640x480' );
   % I_l = imread('saved_l.jpg');
   % I_r = imread('saved_r.jpg');
    [cuttingXYZ, cuttingXYZOver, cuttingXYZUnder, userInputX, userInputY] = XYZ_coordinate_calculation( I_l, I_r, 'calibrationSession.mat', 'stereoParams.mat', ...
        400, 0, 20, 20, firstTgt, userInputX, userInputY);
    
    firstTgt = false;
    
    groupMinIdx = chooseTgt(cuttingXYZ, groupN, lastIdx);
    lastIdx = groupMinIdx;
    
    plot(cuttingXYZ(:,3));
    
    for i = 1:n
        tgt_idx = stepPix*(i-1)+1;
        [tgt_pos, tgt_ori_NOT_USED] = getTgt(tgt_idx,  groupMinIdx, groupN, cuttingXYZ, cuttingXYZOver, cuttingXYZUnder );
        dist_ori = tgt_ori;
        tgt_pos = tgt_pos + [-0.015, 0.0, 0.0]
        
        next_dissection = false;
        while not(next_dissection);
            disp(status.Data);
            sendStatusAck( statusackpub, status.Data )
            gotodistant = false;
            if  i == n
                gotodistant = true;
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
end

sendDone(donepub, true);

rosshutdown;
