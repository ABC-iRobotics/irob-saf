
classdef RetractionControl < handle
    
    properties (Constant)
        FUZZY = 1;
        HMM = 2;
    end
    
    properties
        
        crtl_srv;
        fis;
        method;
        
    end
    
    methods
        
        % Init ------------------------------------------------------------
        function obj = RetractionControl
            
            rosshutdown;
            rosinit;
            
            obj.method = RetractionControl.FUZZY;
            
            obj.fis = readfis('retract_1.fis');
            
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
            % [angle, tension, visible_size]
            
            angle =  reqmsg.Input(1);
            tension =  reqmsg.Input(2);
            visible_size =  reqmsg.Input(3);
            
            displacement = zeros (2,1);
            
            if obj.method == RetractionControl.FUZZY
                
                displacement = evalfis([angle, tension, visible_size],obj.fis);
                
            elseif obj.method == RetractionControl.HMM
                
                % TODO
                
            end

            response.Output(1) = displacement(1);   % y
            response.Output(2) = displacement(2);   % z
            
        end
        
    end
end



