function [ orientation ] = getOri( points3D, disparityMap, stereoParams, im_coord_L, offset_top, offset_center, offset_bottom )

im_coord_L_top = im_coord_L;
im_coord_L_top(:,2) = im_coord_L_top(:,2) + double(offset_top);
points_top = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L_top);

im_coord_L_center = im_coord_L;
im_coord_L_center(:,2) = im_coord_L_center(:,2) + double(offset_center);
points_center = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L_center);

im_coord_L_bottom = im_coord_L;
im_coord_L_bottom(:,2) = im_coord_L_bottom(:,2) + double(offset_bottom);
points_bottom = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L_bottom);

anglesArray = double(zeros(0));

s = min([size(points_top,1) size(points_center,1) size(points_bottom,1)]);


a = double(zeros(s,3));
b = double(zeros(s,3));
c = double(zeros(s,3));

for i = 1:s
    c(i,:) = points_top(i,:);
    a(i,:) = points_bottom(i,:);
    b(i,:) = points_center(i,:);
end



a_mean = double(mean(a,1))
b_mean = double(mean(b,1))
c_mean = double(mean(c,1))

aw = 1.0;
cw = 1.0;

ac_weighted = ((a_mean * aw) + (c_mean * cw)) / (aw + cw);

d = b_mean - ac_weighted;
d(1) = 0.0;

%projection
%disp('ori');
%disp(d);
v = [0.0 0.0 1.0];

R = d' / v';

orientation = rotm2quat(R);

end

