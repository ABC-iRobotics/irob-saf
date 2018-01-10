function [corners,lines, im_foreground] = detect_green_plate(im)

% Returns the corners of the green plate in clockwise order

mask_gp = create_mask_greenplate_2(im);
mask_gp = imcomplement(mask_gp);


se = strel('disk',5);
mask_gp = imopen(mask_gp,se);

mask_gp = imcomplement(bwareafilt(imcomplement(mask_gp),1));

mask_gp3 = cat(3, mask_gp, mask_gp, mask_gp);

im_foreground = im;

im_foreground(mask_gp3) = 0;

edge_gp = edge(mask_gp,'canny');



[H,theta,rho] = hough(edge_gp);

P = houghpeaks(H,4,'threshold',ceil(0.3*max(H(:))));
lines = houghlines(edge_gp,theta,rho,P,'FillGap',80,'MinLength',80);
%figure, imshow(im), hold on


corners = [];
s = size(im);
for k = 1:(length(lines) - 1)
    for l = (k + 1) : length(lines)
        l_intersect_mat = [lines(k).point1; lines(k).point2;...
            lines(l).point1; lines(l).point2];
        p_intersect = linlinintersect(l_intersect_mat);
        
        if (p_intersect(1) > 0) & (p_intersect(2) > 0) &...
                (p_intersect(1) < s(2)) & (p_intersect(2) < s(1))
            corners = [corners; p_intersect];
        end
        
    end
end

if (size(corners, 1) > 0)
    corners = order_poly_cw(corners);
    s = size(im);
    plate_poly_mask = imcomplement(poly2mask(corners(:,1), corners(:,2), s(1), s(2)));
    
    plate_poly_mask3 = cat(3, plate_poly_mask, plate_poly_mask, plate_poly_mask);

    im_foreground = im;

    im_foreground(plate_poly_mask3) = 0;
    
end
%plot(corners(:,1),corners(:,2),'r.');


while (size(corners,1) > 4)
    corners_t = [corners(end,:);corners(1:(end-1),:)];
    diff = abs(corners - corners_t); 
    dist = sum(diff, 2);
    disp(dist);
    disp(size(corners,1));
    [min_dist, min_idx] = min(dist);
    if (min_idx == size(corners,1))
        new_corner = (corners(1,:) + corners(end,:)) /2.0;
        corners = [new_corner;corners(2:end-1,:)];
    elseif min_idx == 1
        new_corner = (corners(1,:) + corners(2,:)) /2.0;
        corners = [new_corner;corners(3:end,:)];
    else
        new_corner = (corners(min_idx,:) + corners(min_idx+1,:)) /2.0;
        corners = [corners(1:min_idx-1,:);new_corner;corners(min_idx+1:end,:)];
    end
end

% TODO remove additional corners

end

