function [ y, z, done ] = fineRetractonCtrl( angle, tension, visible_size )

step = 0.008;
done = true;
y = 0.0;
z = 0.0;

if visible_size < 60.0
    y = y - step;
     done = false;
else
    if and((angle > 0.0),(angle < 45.0)) 
    y = y - step;
    z = z + (step*0.5);
    done = false;
    elseif angle > 120.0
    y = y + (step/2);
    z = z - (step/2);
    done = false;
    end

end

end

