classdef DissectionVision

    
    properties
        groupN = 30;

        tgt_ori = [  0.9938    0.0705    0.0514    0.0685];
        dp_dist = 0.04;
        dp_rot = 345.0;

        dist_pos = [-0.03,-0.0000216146603506,0.20];
        dist_ori = [ 0.9825  ,  0.1830 ,   0.0179 ,  -0.0294];
        
        gotodistant = false;
        stepPix = 10;
        memN = 3;
        lastIdxs = [];
        firstTgt = true;
        userInputX = [0, 0];
        userInputY = [0, 0];
        n = (groupN / stepPix)+1;
        
        targetsrv;
        donesrv; 
        errpub;
        statussub;
        
    end
    
    methods
        
         function obj = DissectionVision
            targetsrv = rossvcserver('/dvrk_vision/movement_target','irob_dvrk_automation/TargetPose', @getTargetCallback)
            donesrv = rossvcserver('/dvrk_vision/task_done', 'irob_dvrk_automation/BoolQuery', @isDoneCallback);
            errpub = rospublisher('/dvrk_vision/error', 'std_msgs/String');

            statussub = rossubscriber('/dvrk_vision/subtask_status', 'std_msgs/String');
            pause(2) % Wait to ensure publisher is registered

            % Start
            status = receive(statussub);
            disp(status.Data);
         end
    end
    
end

