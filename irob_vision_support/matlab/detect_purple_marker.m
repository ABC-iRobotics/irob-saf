function [centroid, im_purple_marker] = detect_white_obj(im)

% Returns the corners of the green plate in clockwise order

mask_wo = create_mask_purple_marker(im);
mask_wo = imcomplement(mask_wo);


se = strel('disk',7);
mask_wo = imopen(mask_wo,se);

mask_wo = imcomplement(bwareafilt(imcomplement(mask_wo),1));

mask_wo3 = cat(3, mask_wo, mask_wo, mask_wo);

im_purple_marker = im;
im_purple_marker(mask_wo3) = 0;

s = regionprops(imcomplement(mask_wo),'centroid');

centroids = cat(1, s.Centroid);

centroid = mean(centroids, 1);


end

