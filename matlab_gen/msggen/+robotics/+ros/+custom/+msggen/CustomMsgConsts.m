classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    properties (Constant)
        irob_msgs_BoolQuery = 'irob_msgs/BoolQuery'
        irob_msgs_BoolQueryRequest = 'irob_msgs/BoolQueryRequest'
        irob_msgs_BoolQueryResponse = 'irob_msgs/BoolQueryResponse'
        irob_msgs_FloatArray = 'irob_msgs/FloatArray'
        irob_msgs_GestureAction = 'irob_msgs/GestureAction'
        irob_msgs_GestureActionFeedback = 'irob_msgs/GestureActionFeedback'
        irob_msgs_GestureActionGoal = 'irob_msgs/GestureActionGoal'
        irob_msgs_GestureActionResult = 'irob_msgs/GestureActionResult'
        irob_msgs_GestureFeedback = 'irob_msgs/GestureFeedback'
        irob_msgs_GestureGoal = 'irob_msgs/GestureGoal'
        irob_msgs_GestureResult = 'irob_msgs/GestureResult'
        irob_msgs_GetControlVariables = 'irob_msgs/GetControlVariables'
        irob_msgs_GetControlVariablesRequest = 'irob_msgs/GetControlVariablesRequest'
        irob_msgs_GetControlVariablesResponse = 'irob_msgs/GetControlVariablesResponse'
        irob_msgs_InstrumentInfo = 'irob_msgs/InstrumentInfo'
        irob_msgs_InstrumentJawPart = 'irob_msgs/InstrumentJawPart'
        irob_msgs_RobotAction = 'irob_msgs/RobotAction'
        irob_msgs_RobotActionFeedback = 'irob_msgs/RobotActionFeedback'
        irob_msgs_RobotActionGoal = 'irob_msgs/RobotActionGoal'
        irob_msgs_RobotActionResult = 'irob_msgs/RobotActionResult'
        irob_msgs_RobotFeedback = 'irob_msgs/RobotFeedback'
        irob_msgs_RobotGoal = 'irob_msgs/RobotGoal'
        irob_msgs_RobotResult = 'irob_msgs/RobotResult'
        irob_msgs_ToolPose = 'irob_msgs/ToolPose'
        irob_msgs_ToolPoseStamped = 'irob_msgs/ToolPoseStamped'
        irob_msgs_TrajectoryToolPose = 'irob_msgs/TrajectoryToolPose'
        irob_msgs_VisionObject = 'irob_msgs/VisionObject'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(25, 1);
                msgList{1} = 'irob_msgs/BoolQueryRequest';
                msgList{2} = 'irob_msgs/BoolQueryResponse';
                msgList{3} = 'irob_msgs/FloatArray';
                msgList{4} = 'irob_msgs/GestureAction';
                msgList{5} = 'irob_msgs/GestureActionFeedback';
                msgList{6} = 'irob_msgs/GestureActionGoal';
                msgList{7} = 'irob_msgs/GestureActionResult';
                msgList{8} = 'irob_msgs/GestureFeedback';
                msgList{9} = 'irob_msgs/GestureGoal';
                msgList{10} = 'irob_msgs/GestureResult';
                msgList{11} = 'irob_msgs/GetControlVariablesRequest';
                msgList{12} = 'irob_msgs/GetControlVariablesResponse';
                msgList{13} = 'irob_msgs/InstrumentInfo';
                msgList{14} = 'irob_msgs/InstrumentJawPart';
                msgList{15} = 'irob_msgs/RobotAction';
                msgList{16} = 'irob_msgs/RobotActionFeedback';
                msgList{17} = 'irob_msgs/RobotActionGoal';
                msgList{18} = 'irob_msgs/RobotActionResult';
                msgList{19} = 'irob_msgs/RobotFeedback';
                msgList{20} = 'irob_msgs/RobotGoal';
                msgList{21} = 'irob_msgs/RobotResult';
                msgList{22} = 'irob_msgs/ToolPose';
                msgList{23} = 'irob_msgs/ToolPoseStamped';
                msgList{24} = 'irob_msgs/TrajectoryToolPose';
                msgList{25} = 'irob_msgs/VisionObject';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(2, 1);
                svcList{1} = 'irob_msgs/BoolQuery';
                svcList{2} = 'irob_msgs/GetControlVariables';
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
                actList = cell(2, 1);
                actList{1} = 'irob_msgs/Gesture';
                actList{2} = 'irob_msgs/Robot';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
