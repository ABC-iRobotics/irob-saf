function [ y, z ] = retractonCtrlProportional(p, angle_des, tension_des, angle, tension)

angle_des_rad = degtorad(angle_des);
tension_des_rad = degtorad(tension_des);

angle_rad = degtorad(angle);
tension_rad = degtorad(tension);

y_norm_des = -(sin(angle_des_rad) + sin(angle_des_rad-tension_des_rad+180));
z_norm_des = -(cos(angle_des_rad) - cos(angle_des_rad-tension_des_rad+180));

y_norm = -(sin(angle_rad) + sin(angle_rad-tension_rad+180));
z_norm = -(cos(angle_rad) - cos(angle_rad-tension_rad+180));


y = p * (y_norm_des - y_norm);
z = p * (z_norm_des - z_norm);



