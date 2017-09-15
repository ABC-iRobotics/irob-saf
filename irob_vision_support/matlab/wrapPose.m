function [ targetmsg ] = wrapPose( targetmsg, tgt_pos, tgt_ori )

    targetmsg.Position.X = tgt_pos(1);
    targetmsg.Position.Y = tgt_pos(2);
    targetmsg.Position.Z = tgt_pos(3);
    targetmsg.Orientation.X =  tgt_ori(2);
    targetmsg.Orientation.Y =  tgt_ori(3);
    targetmsg.Orientation.Z =  tgt_ori(4);
    targetmsg.Orientation.W =  tgt_ori(1) ;


end

