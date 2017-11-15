function [ grab_location, im_coord_L ] = getGrabLocation( IL, IR, disparityMap, P_l, P_r )
   
   
   
    disparityRange = [64,128];
   

    subplot(1,2,1), imshow(IL, [])


    subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
    
   [x,y] = ginput(2);
    minimaArrayX = uint32(x(1)) : uint32(x(2));
    minimaArrayY = (minimaArrayX - minimaArrayX) + uint32(round(mean(y)));
    
    subplot(1,2,1), imshow(IL, [])
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
    hold off

    subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.');
    hold off

    im_coord_L = transpose([minimaArrayX; minimaArrayY ]);
    
    grab_profile = getReconstructedPositions( im_coord_L, disparityMap, P_l, P_r);
    grab_location = mean(grab_profile)
    
    im_coord_L(:,2) = im_coord_L(:,2) - 30;
 
end

