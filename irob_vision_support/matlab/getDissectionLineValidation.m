function [err ] = getDissectionLineValidation( IL, IR, stereoParams )
   
    [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  IL, IR, stereoParams );
    
    [x,y] = ginput(2);
    
    % ground truth
    minimaArrayX_gt = uint32(x(1)) : uint32(x(2));
    step = (y(2) - y(1)) / (size(minimaArrayX_gt) - 1);
    minimaArrayY_gt = y(1) : step : y(2);

    subplot(1,2,2), imshow(ILrect, [])
    hold on
    plot(minimaArrayX_gt, minimaArrayY_gt, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
    hold off

    subplot(1,2,1), imshow(disparityMap,disparityRange);colormap jet
    hold on
    plot(minimaArrayX_gt, minimaArrayY_gt, 'r.');
    hold off

    im_coord_L_gt = transpose([minimaArrayX_gt; minimaArrayY_gt ]);
    
   [ minimaArrayX, minimaArrayY, minimaValues ] = ...
        findDisparityMinimas(disparityMap, im_coord_L_gt, 50, 50 );

    subplot(1,2,2), imshow(ILrect, [])
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
    hold off

    subplot(1,2,1), imshow(disparityMap,disparityRange);colormap jet
    hold on
    plot(minimaArrayX, minimaArrayY, 'r.');
    hold off

    im_coord_L = transpose([minimaArrayX; minimaArrayY ]);
    
    err = minimaArrayY_gt - minimaArrayY;
    
    mean_err = mean(abs(err));
    std_err = std(err);
    
    boxplot(err);
 
end

