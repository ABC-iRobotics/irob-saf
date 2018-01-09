im_L = imread('pnp_test_right.png');


mask_bs = create_mask_bluesheet(im_L);
mask_bs3 = cat(3, mask_bs, mask_bs, mask_bs);

im_L(mask_bs3) = 0;
mask_gp = create_mask_greenplate(im_L);
mask_gp = imcomplement(mask_gp);





se = strel('disk',5);
mask_gp = imopen(mask_gp,se);

mask_gp = imcomplement(bwareafilt(imcomplement(mask_gp),1));


edge_gp = edge(mask_gp,'canny');


[H,theta,rho] = hough(edge_gp);

P = houghpeaks(H,4,'threshold',ceil(0.3*max(H(:))));
lines = houghlines(edge_gp,theta,rho,P,'FillGap',80,'MinLength',80);
figure, imshow(im_L), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   %plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   %plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end

corners = [];
s = size(im_L);
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

plot(corners(:,1),corners(:,2),'r.');


