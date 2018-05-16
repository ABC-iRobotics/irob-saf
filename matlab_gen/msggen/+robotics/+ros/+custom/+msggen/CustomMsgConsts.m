classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    properties (Constant)
        irob_msgs_BoolQuery = 'irob_msgs/BoolQuery'
        irob_msgs_BoolQueryRequest = 'irob_msgs/BoolQueryRequest'
        irob_msgs_BoolQueryResponse = 'irob_msgs/BoolQueryResponse'
        irob_msgs_Environment = 'irob_msgs/Environment'
        irob_msgs_FloatArray = 'irob_msgs/FloatArray'
        irob_msgs_GetControlVariables = 'irob_msgs/GetControlVariables'
        irob_msgs_GetControlVariablesRequest = 'irob_msgs/GetControlVariablesRequest'
        irob_msgs_GetControlVariablesResponse = 'irob_msgs/GetControlVariablesResponse'
        irob_msgs_GraspObject = 'irob_msgs/GraspObject'
        irob_msgs_InstrumentInfo = 'irob_msgs/InstrumentInfo'
        irob_msgs_InstrumentJawPart = 'irob_msgs/InstrumentJawPart'
        irob_msgs_Marker = 'irob_msgs/Marker'
        irob_msgs_MarkerArray = 'irob_msgs/MarkerArray'
        irob_msgs_Point2D = 'irob_msgs/Point2D'
        irob_msgs_RobotAction = 'irob_msgs/RobotAction'
        irob_msgs_RobotActionFeedback = 'irob_msgs/RobotActionFeedback'
        irob_msgs_RobotActionGoal = 'irob_msgs/RobotActionGoal'
        irob_msgs_RobotActionResult = 'irob_msgs/RobotActionResult'
        irob_msgs_RobotFeedback = 'irob_msgs/RobotFeedback'
        irob_msgs_RobotGoal = 'irob_msgs/RobotGoal'
        irob_msgs_RobotResult = 'irob_msgs/RobotResult'
        irob_msgs_SurgemeAction = 'irob_msgs/SurgemeAction'
        irob_msgs_SurgemeActionFeedback = 'irob_msgs/SurgemeActionFeedback'
        irob_msgs_SurgemeActionGoal = 'irob_msgs/SurgemeActionGoal'
        irob_msgs_SurgemeActionResult = 'irob_msgs/SurgemeActionResult'
        irob_msgs_SurgemeFeedback = 'irob_msgs/SurgemeFeedback'
        irob_msgs_SurgemeGoal = 'irob_msgs/SurgemeGoal'
        irob_msgs_SurgemeResult = 'irob_msgs/SurgemeResult'
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
                msgList = cell(30, 1);
                msgList{1} = 'irob_msgs/BoolQueryRequest';
                msgList{2} = 'irob_msgs/BoolQueryResponse';
                msgList{3} = 'irob_msgs/Environment';
                msgList{4} = 'irob_msgs/FloatArray';
                msgList{5} = 'irob_msgs/GetControlVariablesRequest';
                msgList{6} = 'irob_msgs/GetControlVariablesResponse';
                msgList{7} = 'irob_msgs/GraspObject';
                msgList{8} = 'irob_msgs/InstrumentInfo';
                msgList{9} = 'irob_msgs/InstrumentJawPart';
                msgList{10} = 'irob_msgs/Marker';
                msgList{11} = 'irob_msgs/MarkerArray';
                msgList{12} = 'irob_msgs/Point2D';
                msgList{13} = 'irob_msgs/RobotAction';
                msgList{14} = 'irob_msgs/RobotActionFeedback';
                msgList{15} = 'irob_msgs/RobotActionGoal';
                msgList{16} = 'irob_msgs/RobotActionResult';
                msgList{17} = 'irob_msgs/RobotFeedback';
                msgList{18} = 'irob_msgs/RobotGoal';
                msgList{19} = 'irob_msgs/RobotResult';
                msgList{20} = 'irob_msgs/SurgemeAction';
                msgList{21} = 'irob_msgs/SurgemeActionFeedback';
                msgList{22} = 'irob_msgs/SurgemeActionGoal';
                msgList{23} = 'irob_msgs/SurgemeActionResult';
                msgList{24} = 'irob_msgs/SurgemeFeedback';
                msgList{25} = 'irob_msgs/SurgemeGoal';
                msgList{26} = 'irob_msgs/SurgemeResult';
                msgList{27} = 'irob_msgs/ToolPose';
                msgList{28} = 'irob_msgs/ToolPoseStamped';
                msgList{29} = 'irob_msgs/TrajectoryToolPose';
                msgList{30} = 'irob_msgs/VisionObject';
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
                actList{1} = 'irob_msgs/Robot';
                actList{2} = 'irob_msgs/Surgeme';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
