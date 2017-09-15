classdef DissectionVisionValidation < handle
    
    
    properties
        
        stereoParams;
       
        dissection_profile;
        im_coord_L;
        
        cam_l;
        cam_r;
        
    end
    
    methods
        
        function obj = DissectionVisionValidation
          
            obj.stereoParams=load('stereoParams.mat');
            imaqreset;
            webcamlist()
            obj.cam_r = videoinput('linuxvideo', 1, 'BGR24_640x480');
            obj.cam_l = videoinput('linuxvideo', 2, 'BGR24_640x480');
            
            preview([obj.cam_l, obj.cam_r]);
           % triggerconfig(obj.cam_r, 'manual');
            %triggerconfig(obj.cam_l, 'manual');
           % start(obj.cam_r);
            %start(obj.cam_l);
            set_focus;
            %toc
            pause(1);
            
           [ I_l, I_r ] = stereo_capture(obj.cam_l, obj.cam_r);
                    
           getDissectionLineValidation( I_l, I_r, obj.stereoParams, obj.im_coord_L );
                    
                   
        end
        
    end
end

