function [ dp_pos, dp_ori ] = getDP(dp_dist, dp_rot, tgt_pos, tgt_ori )

    dp_rot_euler = [0.0 0.0 degtorad(dp_rot)];
    RR = eul2rotm(dp_rot_euler, 'ZYX');
            
    R = quat2rotm(tgt_ori);
    R = RR * R;
    v = [0.0 0.0 1.0];
    v = (R * v')';
            
    dp_pos = tgt_pos - (dp_dist * v);
    dp_ori_R = quat2rotm(tgt_ori);
    dp_ori_R = RR  * dp_ori_R;
    dp_ori = rotm2quat(dp_ori_R);

end

