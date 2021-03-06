classdef GetControlVariablesRequest < ros.Message
    %GetControlVariablesRequest MATLAB implementation of irob_msgs/GetControlVariablesRequest
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'irob_msgs/GetControlVariablesRequest' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '806a17ef362f358a69865795ac295dc8' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Input
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Input'} % List of non-constant message properties
        ROSPropertyList = {'input'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = GetControlVariablesRequest(msg)
            %GetControlVariablesRequest Construct the message object GetControlVariablesRequest
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
        
        function input = get.Input(obj)
            %get.Input Get the value for property Input
            javaArray = obj.JavaMessage.getInput;
            array = obj.readJavaArray(javaArray, 'double');
            input = double(array);
        end
        
        function set.Input(obj, input)
            %set.Input Set the value for property Input
            if ~isvector(input) && isempty(input)
                % Allow empty [] input
                input = double.empty(0,1);
            end
            
            validateattributes(input, {'numeric'}, {'vector'}, 'GetControlVariablesRequest', 'Input');
            
            javaArray = obj.JavaMessage.getInput;
            array = obj.writeJavaArray(input, javaArray, 'double');
            obj.JavaMessage.setInput(array);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Input = obj.Input;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Input = strObj.Input;
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
            
            strObj.Input = obj.Input;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.irob_msgs.GetControlVariablesRequest.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.irob_msgs.GetControlVariablesRequest;
            obj.reload(strObj);
        end
    end
end
