function [ ] = sendDone( donepub, done )

donemsg = rosmessage(donepub);
donemsg.Data = done;
send(donepub,donemsg);

end

