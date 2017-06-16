function [ dissectionProfile, orientation, im_coord_L ] = getDissectionProfileAndOri( IL, IR, stereoParams, prev_im_coord_L )
    
    [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  IL, IR, stereoParams );
    
    %in pixs
    [ y_top, y_bottom ] = phantomSegmentation( IL, IR, disparityMap, prev_im_coord_L);
    
    prev_y_mean = mean(prev_im_coord_L(:,2));
    top_margin = abs(y_top - prev_y_mean) * 0.5; 
    bottom_margin = abs(y_bottom - prev_y_mean) * 0.5;
    
    [ minimaArrayX, minimaArrayY, minimaValues ] = ...
        findDisparityMinimas(disparityMap, prev_im_coord_L, top_margin, bottom_margin );

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
    
    points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
    points3D = points3D ./ 1000;
    
    dissectionProfile = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L);
 
    orientation = getOri(points3D, disparityMap, stereoParams,  im_coord_L, -(abs(y_top - y_mean) * 0.5), 0, -(abs(y_bottom - y_mean) * 0.5));
    
end

