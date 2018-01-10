IL = imread('pnp_test_right.png');

[corners_L, lines_L, im_foreground_L] = detect_green_plate(IL);

%disparityMap = readImage(disparity_msg.Image);

%imshow([IL, IR])


%grab_pos = ...
%    triangulate(IL_centroid,IR_centroid,left_p,right_p) * 1000.0             % in mm

%         tgt_msg = rosmessage(target_pub);
%         tgt_msg.X = grab_pos(1);
%         tgt_msg.Y = grab_pos(2);
%         tgt_msg.Z = grab_pos(3);
%         send(target_pub,tgt_msg);

if (size(corners_L) > 0)
    
    s = size(IL);
    plate_poly_mask_L = imcomplement(poly2mask(corners_L(:,1), corners_L(:,2), s(1), s(2)));
    
    plate_poly_mask_L3 = cat(3, plate_poly_mask_L, plate_poly_mask_L, plate_poly_mask_L);
    im_foreground_L(plate_poly_mask_L3) = 0;
    [centroid, im_white_object] = detect_white_obj(IL);
    
    subplot(1,2,2), ...
    imshow(im_white_object)
    hold on
    plot(centroid(1),centroid(2),'g.');
    hold off
   
    subplot(1,2,1), ...
    imshow(im_foreground_L)
    hold on
  
    for i = 1:length(lines_L)
        xy = [lines_L(i).point1; lines_L(i).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    end
    plot(corners_L(:,1),corners_L(:,2),'r.');
    hold on
    
    [centroid, im_white_object] = detect_white_obj(IL);

    plot(centroid(1),centroid(2),'g.');
end



    
