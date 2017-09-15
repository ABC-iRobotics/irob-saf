function [ angle, tension, visible_size, im_coord_L ] = getRetractionAngles( IL, IR, stereoParams, prev_im_coord_L )
    %disp('disp map');
    %tic
    [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  IL, IR, stereoParams );
    %toc
    %in pixs
    [ y_top, y_bottom ] = phantomSegmentation( IL, IR, disparityMap,prev_im_coord_L );
    y_top = double(y_top);
     y_bottom = double(y_bottom);
    visible_size = abs(y_bottom - y_top);
    
    prev_y_mean = mean(prev_im_coord_L(2,:));
    top_margin = abs(y_top - prev_y_mean) * 0.2; 
    bottom_margin = abs(y_bottom - prev_y_mean) * 0.2;
    
   % disp('minimas');
    %tic
    [ minimaArrayX, minimaArrayY, minimaValues ] = ...
        findDisparityMinimas(disparityMap, prev_im_coord_L, top_margin, bottom_margin );
   % toc
    
    subplot(1,2,2), imshow(ILrect, [])
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
    hold off

    subplot(1,2,1), imshow(disparityMap,disparityRange);colormap jet
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.');
    hold off

    im_coord_L = transpose([minimaArrayX; minimaArrayY ]);
    y_mean = mean(im_coord_L(2,:));
    
   % disp('3D reconstruction');
   % tic
    points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
    points3D = points3D ./ 1000;
  %  toc
    
  %  disp('angles');
   % tic
    angle  = getAngle( points3D, disparityMap, stereoParams,  im_coord_L, -(double(abs(y_top - y_mean)) * 0.5), 0, -(double(abs(y_bottom - y_mean)) * 0.5) );
    
    tension = getAngle( points3D, disparityMap, stereoParams, im_coord_L, -(double(abs(y_top - y_mean)) * 0.8), -(double(abs(y_top - y_mean)) * 0.4), 0 );
   % toc
end

