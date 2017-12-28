function [ positions ] = getReconstructedPositions(  disparityMap, P_l, P_r,im_coord_L)
    
    sizeDM = size(disparityMap);

    for i = 1: size(im_coord_L, 1)
        if im_coord_L(i,1) < 1
            im_coord_L(i,1) = 1;
        elseif im_coord_L(i,1) > sizeDM(2)
            im_coord_L(i,1) = sizeDM(2);
        end

         if im_coord_L(i,2) < 1
            im_coord_L(i,2) = 1;
        elseif im_coord_L(i,2) > sizeDM(1)
            im_coord_L(i,2) = sizeDM(1);
        end
    end

    im_coord_R = uint32(zeros(size(im_coord_L, 1), 2));


    for i = 1:size(im_coord_L, 1)
        im_coord_R(i,:) = [im_coord_L(i,1) - uint32(disparityMap(im_coord_L(i,2), im_coord_L(i,1))),im_coord_L(i,2)];    
    end
    
    %disp(im_coord_R);
   
    cuttingXYZ = triangulate(uint32(im_coord_L), uint32(im_coord_R), P_l, P_r);

    positions = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);

end

