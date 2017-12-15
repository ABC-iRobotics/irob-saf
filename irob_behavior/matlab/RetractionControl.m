
classdef RetractionControl < handle
    
    properties (Constant)
        FUZZY = 1;
        HMM = 2;
        STRAIGHT = 3;
    end
    
    properties
        
        crtl_srv;
        fis;
        method;
        
        angles;
        tensions;
        visible_sizes;
                
    end
    
    methods
        
        % Init ------------------------------------------------------------
        function obj = RetractionControl
            
            rosshutdown;
            rosinit;
            
            obj.angles = double(zeros(0));
            obj.tensions = double(zeros(0));
            obj.visible_sizes = double(zeros(0));
           
            
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
            
            obj.angles =  [obj.angles reqmsg.Input(1)];
            obj.tensions =   [obj.tensions reqmsg.Input(2)];
            obj.visible_sizes =  [obj.visible_sizes reqmsg.Input(3)];
            
            displacement = zeros (2,1);
            
            if obj.method == RetractionControl.FUZZY
                
                [ y, z ] = retractonCtrlFuzzy(obj.fis, ...
                       obj.angles(end), ...
                       obj.tensions(end), obj.visible_sizes(end));
                
            elseif obj.method == RetractionControl.HMM
                
                [ y, z ] = retractonCtrlHMM( obj.angles, obj.tensions, obj.visible_sizes );

            elseif obj.method == RetractionControl.STRAIGHT
                
                % TODO
                    
            end

            response.Output(1) = y;   % y
            response.Output(2) = z;   % z
            
        end
        
    end
end



