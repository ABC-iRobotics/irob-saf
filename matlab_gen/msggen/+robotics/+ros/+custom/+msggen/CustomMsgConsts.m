classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    properties (Constant)
        irob_dvrk_automation_BoolQuery = 'irob_dvrk_automation/BoolQuery'
        irob_dvrk_automation_BoolQueryRequest = 'irob_dvrk_automation/BoolQueryRequest'
        irob_dvrk_automation_BoolQueryResponse = 'irob_dvrk_automation/BoolQueryResponse'
        irob_dvrk_automation_TargetPose = 'irob_dvrk_automation/TargetPose'
        irob_dvrk_automation_TargetPoseRequest = 'irob_dvrk_automation/TargetPoseRequest'
        irob_dvrk_automation_TargetPoseResponse = 'irob_dvrk_automation/TargetPoseResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(4, 1);
                msgList{1} = 'irob_dvrk_automation/BoolQueryRequest';
                msgList{2} = 'irob_dvrk_automation/BoolQueryResponse';
                msgList{3} = 'irob_dvrk_automation/TargetPoseRequest';
                msgList{4} = 'irob_dvrk_automation/TargetPoseResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(2, 1);
                svcList{1} = 'irob_dvrk_automation/BoolQuery';
                svcList{2} = 'irob_dvrk_automation/TargetPose';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
