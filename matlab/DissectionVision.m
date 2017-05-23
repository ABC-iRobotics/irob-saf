classdef DissectionVision
    
    
    properties
        
        state = DissectionStates.done_dissection_group
        
        dissection_group_n = 0;
        local_dissection_n = 0;
        
        
        task_done = false;
        
        groupN = 30;
        
        tgt_ori = [  0.9938    0.0705    0.0514    0.0685];
        dp_dist = 0.04;
        dp_rot = 345.0;
        
        dist_pos = [-0.03,-0.0000216146603506,0.20];
        dist_ori = [ 0.9825  ,  0.1830 ,   0.0179 ,  -0.0294];
        
        
        stepPix = 10;
        memN = 3;
        lastIdxs = [];
        groupMinIdx = 1;
        firstTgt = true;
        userInputX = [0, 0];
        userInputY = [0, 0];
        n;
        stereoParams;
        tgt_idx = 1;
        tgt_pos;
        cuttingXYZ;
        
        targetsrv;
        donesrv;
        errpub;
        statussub;
        
    end
    
    methods
        
        function obj = DissectionVision
            obj.targetsrv = rossvcserver('/dvrk_vision/movement_target','irob_dvrk_automation/TargetPose', @obj.getTargetCallback)
            obj.donesrv = rossvcserver('/dvrk_vision/task_done', 'irob_dvrk_automation/BoolQuery', @obj.isDoneCallback);
            obj.errpub = rospublisher('/dvrk_vision/error', 'std_msgs/String');
            
            obj.statussub = rossubscriber('/dvrk_vision/subtask_status', 'std_msgs/String');
            pause(2) % Wait to ensure publisher is registered
            
            % Start
            status = receive(statussub);
            disp(status.Data);
            
            obj.stereoParams= load('stereoParams.mat');
            obj.n = (groupN / stepPix)+1;
        end
        
        
        
        % Callback for pos query
        function response = getTargetCallback(obj,server,reqmsg,defaultrespmsg)
            response = defaultrespmsg;
            % Build the response message here
            
            group_done = false;
            
            switch obj.state
                case DissectionStates.done_dissection_group
                    % capture img; choose group loc; choose tgt; get dp; goto tgt dp;
                    %[ I_r, I_l ] = stereo_capture('BGR24_640x480', obj.firstTgt);
                     I_l = imread('saved_l.jpg');
                     I_r = imread('saved_r.jpg');
                    
                     obj.firstTgt = false;
                    
                    [obj.cuttingXYZ, cuttingXYZOver, cuttingXYZUnder, userInputX, userInputY] = ...
                        XYZ_coordinate_calculation( I_l, I_r, obj.stereoParams, ...
                        400, 0, 25, 25, firstTgt, userInputX, userInputY);
                    
                    [obj.groupMinIdx, obj.lastIdxs] = chooseTgt(obj.cuttingXYZ, obj.groupN, obj.lastIdxs, obj.memN);
                    
                    obj.dissection_group_n = obj.dissection_group_n + 1;
                    obj.local_dissection_n = 0;
                    
                    obj.tgt_idx = obj.stepPix*obj.local_dissection_n +1;
                    [obj.tgt_pos, tgt_ori_NOT_USED] = getTgt(obj.tgt_idx, ...
                        obj.groupMinIdx, obj.groupN, obj.cuttingXYZ, obj.cuttingXYZOver, obj.cuttingXYZUnder );
                    
                    obj.dist_ori = obj.tgt_ori;
                    obj.tgt_pos = obj.tgt_pos + [-0.015, 0.0, 0.0];
                    
                    [dp_pos, dp_ori] = getDP(obj.dp_dist, obj.dp_rot, obj.tgt_pos, obj.tgt_ori );
                    
                    response = obj.wrapPose(response, dp_pos, dp_ori)
                    response.position_type = response.DP;
                    
                    
                case DissectionStates.at_tgt_dp
                    % goto tgt as goal
                    response = obj.wrapPose(response, obj.tgt_pos, obj.tgt_ori)
                    response.position_type = response.GOAL;
                    
                case DissectionStates.at_tgt_goal
                    % goto distant dp as goal
                    
                    % or go to distant dp as dp
                    obj.local_dissection_n = obj.local_dissection_n + 1;
                    
                    [dp_pos, dp_ori] = getDP(obj.dp_dist, obj.dp_rot, obj.tgt_pos, obj.tgt_ori );
                    
                    response = obj.wrapPose(response, dp_pos, dp_ori)
                    
                    if obj.local_dissection_n >= obj.n
                        if obj.dissection_group_n >= obj.groupN
                            obj.task_done = true;
                        end
                        group_done = true;
                        response.position_type = response.DP;
                    else
                        group_done = false;
                        response.position_type = response.GOAL;
                    end
                    
                case DissectionStates.at_distant_dp
                    %go to distant goal
                    response = obj.wrapPose(response, obj.dist_pos, obj.dist_ori);
                    response.position_type = response.GOAL;
                    
                case DissectionStates.at_distant_goal
                    % choose tgt; get dp; go to tgt dp
                    obj.tgt_idx = obj.stepPix*obj.local_dissection_n +1;
                    [obj.tgt_pos, tgt_ori_NOT_USED] = getTgt(obj.tgt_idx, ...
                        obj.groupMinIdx, obj.groupN, obj.cuttingXYZ, obj.cuttingXYZOver, obj.cuttingXYZUnder );
                    
                    obj.dist_ori = obj.tgt_ori;
                    obj.tgt_pos = obj.tgt_pos + [-0.015, 0.0, 0.0];
                    
                    [dp_pos, dp_ori] = getDP(obj.dp_dist, obj.dp_rot, obj.tgt_pos, obj.tgt_ori );
                    
                    response = obj.wrapPose(response, dp_pos, dp_ori)
                    response.position_type = response.DP;
                    
                otherwise
                    warning('Unexpected query, do nothing...')
            end
            
            % Step state
            obj.state = obj.state.next(reqmsg, group_done);
            
            if obj.state == DissectionStates.abort
                response = defaultrespmsg;
                % Do err handling
            end
            
        end
        
        
        
        % Callback for task done
        function response = isDoneCallback(obj,server,reqmsg,defaultrespmsg)
            response = defaultrespmsg;
            % Build the response message here
            
            response.data = obj.task_done;
        end
        
        function response = wrapPose(response, pos, ori)
            response.pose.position.x = pos(1);
            response.pose.Position.X = pos(1);
            response.pose.Position.Y = pos(2);
            response.pose.Position.Z = pos(3);
            response.pose.Orientation.X =  ori(2);
            response.pose.Orientation.Y =  ori(3);
            response.pose.Orientation.Z =  ori(4);
            response.pose.Orientation.W =  ori(1) ;
        end
    end
    
end

