
classdef RetractionControl < handle
    
    
    properties
        
        crtl_srv;
        
    end
    
    methods
        
        % Init ------------------------------------------------------------
        function obj = RetractionControl
            
            rosshutdown;
            rosinit;
            
            obj.crtl_srv = rossvcserver(...
                '/ias/behavior/retract_crtl_srv',...
                'irob_msgs/GetControlVariables', @obj.getControlVariables)
            
            pause(2) % Wait to ensure publisher is registered
            
            
        end
        
        
        % Callback for crtl query -----------------------------------------
        function response = getControlVariables(...
                obj,server,reqmsg,defaultrespmsg)
            
            response = defaultrespmsg;
            
            % Build the response message here
            
            response.Output(1) = reqmsg.Input(1) * 2.0;
            response.Output(2) = reqmsg.Input(2) * 2.0;
            
        end
        
    end
end



