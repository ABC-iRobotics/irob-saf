function [ y, z ] = retractonCtrlFuzzy( fis, angle, tension, visible_size )

y = 0.0;
z = 0.0;

output = evalfis(double([angle, tension, visible_size]),fis);
y = output(1)
z = output(2)


end

