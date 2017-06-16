function [ y, z, done ] = fineRetractonCtrlFuzzy( angle, tension, visible_size )

step = 0.008;
done = true;
y = 0.0;
z = 0.0;

fis = readfis('retract_1.fis');
output = evalfis(double([angle, tension, visible_size]),fis);
y = output(1)
z = output(2)



end

