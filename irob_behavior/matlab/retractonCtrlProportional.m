function [ y, z ] = retractonCtrlProportional(p, angle_des, tension_des, angle, tension)

angle_des_rad = degtorad(angle_des);
tension_des_rad = degtorad(tension_des);

angle_rad = degtorad(angle);
tension_rad = degtorad(tension);

y_norm_des = -(sin(angle_des_rad) + sin(tension_des_rad)) / 2.0;
z_norm_des = -(cos(angle_des_rad) - cos(tension_des_rad)) / 2.0;

y_norm = -(sin(angle_rad) + sin(tension_rad)) / 2.0;
z_norm = -(cos(angle_rad) - cos(tension_rad)) / 2.0;


y = p * (y_norm_des - y_norm);
z = p * (z_norm_des - z_norm);



