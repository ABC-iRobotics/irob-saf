set_focus;
%toc
pause(1);

[ I_l, I_r ] = stereo_capture(cam_l,cam_r);
err{end + 1} = getDissectionLineValidation( I_l, I_r, stereoParams);

