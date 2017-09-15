function [ output_args ] = sendStatusAck( statusackpub, status )

    ackmsg = rosmessage(statusackpub);
    ackmsg.Data = strcat(status, '_ack');
    send(statusackpub,ackmsg);


end

