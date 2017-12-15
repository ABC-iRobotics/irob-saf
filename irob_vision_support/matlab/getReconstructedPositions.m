function [ positions ] = getReconstructedPositions(  disparityMap, P_l, P_r,im_coord_L)
    
    im_coord_R = uint32(zeros(size(im_coord_L, 1), 2));
    
    for i = 1:size(im_coord_L, 2)
        im_coord_R(i,:) = [im_coord_L(i,1) - uint32(disparityMap(im_coord_L(i,2), im_coord_L(i,1))),im_coord_L(i,2)];    
    end
    
    disp(im_coord_R);
   
    cuttingXYZ = triangulate(im_coord_L, im_coord_R, P_l, P_r);

    positions = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);

end

