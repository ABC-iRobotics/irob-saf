function [ output_args ] = goToPos(statussub, statusackpub, targetpub, targettypepub, targetvalidpub, type, pos, ori )

%disp(pos);
status = statussub.LatestMessage;
while not(strcmp(status.Data,'waiting_for_target'))
    pause(0.1);
    status = statussub.LatestMessage;
end
validmsg = rosmessage(targetvalidpub);
validmsg.Data = false;
send(targetvalidpub,validmsg);
sendStatusAck( statusackpub, status.Data);

targetmsg = rosmessage(targetpub);
targetmsg = wrapPose(targetmsg, pos, ori);
send(targetpub,targetmsg);

typemsg = rosmessage(targettypepub);
typemsg.Data = type;
send(targettypepub,typemsg);

validmsg = rosmessage(targetvalidpub);
validmsg.Data = true;
send(targetvalidpub,validmsg);

status = statussub.LatestMessage;

while not(or(strcmp(status.Data,'dp_reached'), ...
        or(strcmp(status.Data,'goal_reached'), strcmp(status.Data,'abort'))))
    % TODO err handling
    send(targetpub,targetmsg);
    pause(0.1);
    status = statussub.LatestMessage;
end
validmsg = rosmessage(targetvalidpub);
validmsg.Data = false;
send(targetvalidpub,validmsg);
sendStatusAck( statusackpub, status.Data);



pause(0.01);

end

