set_focus;
%toc
pause(1);

[ I_l, I_r ] = stereo_capture(obj.cam_l, obj.cam_r);
err{end + 1} = getDissectionLineValidation( I_l, I_r, obj.stereoParams, obj.im_coord_L );

