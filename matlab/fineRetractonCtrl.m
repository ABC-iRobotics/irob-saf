function [ y, z, done ] = fineRetractonCtrl( angle, tension, visible_size )

step = 0.008;
done = true;
y = 0.0;
z = 0.0;

if (angle > 0.0)
    y = y +  sin((angle * 180.00)/pi)*step*(-1);
    z = z - cos((angle * 180.00)/pi)*step;
    done = false;
elseif (angle > 120.0)
    y = y -  sin((angle * 180.00)/pi)*step*(-1);
    z = z + cos((angle * 180.00)/pi)*step;
    done = false;
elseif (tension < 250.0)
    y = y + cos((angle * 180.00)/pi)*step*(-1);
    z = z + sin((angle * 180.00)/pi)*step*(-1);
    done = false;
elseif (tension > 0.0)
    y = y - cos((angle * 180.00)/pi)*step*(-1);
    z = z - sin((angle * 180.00)/pi)*step*(-1);
    done = false;
end


end

