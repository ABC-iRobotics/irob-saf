classdef RobotResult < ros.Message
    %RobotResult MATLAB implementation of irob_msgs/RobotResult
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'irob_msgs/RobotResult' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'bf574a43d53221758a40ca6057f570e0' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        IrobMsgsToolPoseClass = ros.msg.internal.MessageFactory.getClassForType('irob_msgs/ToolPose') % Dispatch to MATLAB class for message type irob_msgs/ToolPose
    end
    
    properties (Dependent)
        Pose
        Info
    end
    
    properties (Access = protected)
        Cache = struct('Pose', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Info', 'Pose'} % List of non-constant message properties
        ROSPropertyList = {'info', 'pose'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = RobotResult(msg)
            %RobotResult Construct the message object RobotResult
            import com.mathworks.toolbox.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('ros:mlros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('ros:mlros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('ros:mlros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function pose = get.Pose(obj)
            %get.Pose Get the value for property Pose
            if isempty(obj.Cache.Pose)
                obj.Cache.Pose = feval(obj.IrobMsgsToolPoseClass, obj.JavaMessage.getPose);
            end
            pose = obj.Cache.Pose;
        end
        
        function set.Pose(obj, pose)
            %set.Pose Set the value for property Pose
            validateattributes(pose, {obj.IrobMsgsToolPoseClass}, {'nonempty', 'scalar'}, 'RobotResult', 'Pose');
            
            obj.JavaMessage.setPose(pose.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Pose)
                obj.Cache.Pose.setJavaObject(pose.getJavaObject);
            end
        end
        
        function info = get.Info(obj)
            %get.Info Get the value for property Info
            info = char(obj.JavaMessage.getInfo);
        end
        
        function set.Info(obj, info)
            %set.Info Set the value for property Info
            info = convertStringsToChars(info);
            
            validateattributes(info, {'char', 'string'}, {}, 'RobotResult', 'Info');
            
            obj.JavaMessage.setInfo(info);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Pose = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Info = obj.Info;
            
            % Recursively copy compound properties
            cpObj.Pose = copy(obj.Pose);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Info = strObj.Info;
            obj.Pose = feval([obj.IrobMsgsToolPoseClass '.loadobj'], strObj.Pose);
        end
    end
    
    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Info = obj.Info;
            strObj.Pose = saveobj(obj.Pose);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.irob_msgs.RobotResult.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.irob_msgs.RobotResult;
            obj.reload(strObj);
        end
    end
end
