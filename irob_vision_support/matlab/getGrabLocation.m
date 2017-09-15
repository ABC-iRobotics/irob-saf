function [ grab_location, im_coord_L ] = getGrabLocation( IL, IR, stereoParams )
   
    [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  IL, IR, stereoParams );
    
    [x,y] = ginput(2);
    
    minimaArrayX = uint32(x(1)) : uint32(x(2));
    minimaArrayY = (minimaArrayX - minimaArrayX) + uint32(round(mean(y)));

    subplot(1,2,2), imshow(ILrect, [])
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
    hold off

    subplot(1,2,1), imshow(disparityMap,disparityRange);colormap jet
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.');
    hold off

    im_coord_L = transpose([minimaArrayX; minimaArrayY ]);
  
    points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
    points3D = points3D ./ 1000;
    
    grab_profile = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L);
    grab_location = mean(grab_profile)
    
    im_coord_L(:,2) = im_coord_L(:,2) - 30;
 
end

