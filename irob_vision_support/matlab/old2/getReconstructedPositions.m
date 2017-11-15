function [ positions ] = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L)

    cuttingXYZIdx = uint32(round(sub2ind(size(disparityMap), im_coord_L(:, 2), im_coord_L(:, 1))));
    X = points3D(:, :, 1);
    Y = points3D(:, :, 2);
    Z = points3D(:, :, 3);
    cuttingXYZ = [X(cuttingXYZIdx)'; Y(cuttingXYZIdx)'; Z(cuttingXYZIdx)']';
    positions = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);

end

