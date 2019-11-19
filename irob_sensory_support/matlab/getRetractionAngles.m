function [ angle, tension, visible_size, im_coord_L ] = getRetractionAngles( disparityMap,  P_l, P_r, prev_im_coord_L )

    [ y_top, y_bottom ] = phantomSegmentation(disparityMap,prev_im_coord_L );
    y_top = double(y_top);
     y_bottom = double(y_bottom);
    visible_size = abs(y_bottom - y_top);
    
    prev_y_mean = mean(prev_im_coord_L(:,2));
    %top_margin = abs(y_top - prev_y_mean) * 0.2; 
    %bottom_margin = abs(y_bottom - prev_y_mean) * 0.4;
    
    top_margin = 60; 
    bottom_margin = 60;

    [ minimaArrayX, minimaArrayY, minimaValues ] = ...
        findDisparityMinimas(disparityMap, prev_im_coord_L, top_margin, bottom_margin );
   
%    subplot(1,2,2), imshow(ILrect, [])
 %   hold on
  %  plot(minimaArrayX, minimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
 %   hold off

  %  subplot(1,2,1), imshow(disparityMap,disparityRange);colormap jet
  %  hold on
  %  plot(minimaArrayX, minimaArrayY, 'r.');
  %  hold off

    im_coord_L = transpose([minimaArrayX; minimaArrayY ]);
    y_mean = mean(im_coord_L(:,2));
    

    angle  = getAngle( disparityMap,  P_l, P_r, ...
        im_coord_L,...
        -(double(abs(y_top - y_mean)) * 0.3), ...
        0, ...
        (double(abs(y_bottom - y_mean)) * 0.3) );
    
    tension = getAngle( disparityMap,  P_l, P_r,...
        im_coord_L, ....
        -(double(abs(y_top - y_mean)) * 0.6),...
        -(double(abs(y_top - y_mean)) * 0.3),...
        0 );
end

