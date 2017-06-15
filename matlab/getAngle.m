function [ mean_angle ] = getAngle( points3D, disparityMap, stereoParams, im_coord_L, offset_top, offset_center, offset_bottom )

    im_coord_L_top = im_coord_L;
    im_coord_L_top(:,2) = im_coord_L_top(:,2) + offset_top;
    points_top = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L_top);
    
    im_coord_L_center = im_coord_L;
    im_coord_L_center(:,2) = im_coord_L_center(:,2) + offset_center;
    points_center = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L_center);
    
    im_coord_L_bottom = im_coord_L;
    im_coord_L_bottom(:,2) = im_coord_L_bottom(:,2) + offset_bottom;
    points_bottom = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L_bottom);

    anglesArray = double(zeros(0));
for i = 1:min([size(points_top(:,1)),size(points_center(:,1)),size(points_bottom(:,1))])
    c = points_top(i,:);
    a = points_bottom(i,:);
    b = points_center(i,:);
 
    v1 = [a(:,1) - b(:,1), a(:,2) - b(:,2), a(:,3) - b(:,3)];
    v2 = [c(:,1) - b(:,1), c(:,2) - b(:,2), c(:,3) - b(:,3)];
    v3 = [a(:,1) - c(:,1), a(:,2) - c(:,2), a(:,3) - c(:,3)];

    v1mag = sqrt(abs(v1(1) * v1(1) + v1(2) * v1(2) + v1(3) * v1(3)));
    v2mag = sqrt(abs(v2(1) * v2(1) + v2(2) * v2(2) + v2(3) * v2(3)));
    v3mag = sqrt(abs(v3(1) * v3(1) + v3(2) * v3(2) + v3(3) * v3(3)));

    angle = acos(((v2mag * v2mag) + (v1mag * v1mag) - (v3mag * v3mag))/ (2*v2mag*v1mag));
    angle = angle * (180/pi);
    anglesArray = [anglesArray, angle];
end

    mean_angle = mean(anglesArray);

end

