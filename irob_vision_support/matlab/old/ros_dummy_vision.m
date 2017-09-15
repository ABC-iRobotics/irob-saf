clear all;

rosinit;

[targetpub, targetvalidpub, targettypepub, statusackpub, donepub, errpub, statussub] ...
        = dvrkInit();

tgt_pos = [0.1 0.0 0.4];
tgt_ori = [ -0.9825   , 0.1830 ,   -0.0179 ,  -0.0294];

dp_dist = 0.04;
dp_rot = -15.0;

dist_pos = [0.1 -0.05 0.35];
dist_ori = [ 0.9825   , 0.1830 ,   0.0179 ,  -0.0294];
status = statussub.LatestMessage;

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

sendDone(donepub, true);

rosshutdown;



