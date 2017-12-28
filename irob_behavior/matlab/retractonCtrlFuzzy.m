function [ y, z ] = retractonCtrlFuzzy( fis, angle, tension )

y = 0.0;
z = 0.0;

output = evalfis(double([angle, tension]),fis);
y = output(1)
z = output(2)


end

