function [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  IL, IR, stereoParams )

maxDisp = 400;

%Rectify the images
[ILrect, IRrect] = ...
rectifyStereoImages(IL, IR, stereoParams.stereoParams);

frameLeftGray  = rgb2gray(ILrect);
frameRightGray = rgb2gray(IRrect);


disparityRange = [0,maxDisp];
disparityMap = disparity(frameLeftGray,frameRightGray,'BlockSize',...
   15,'DisparityRange',disparityRange, 'ContrastThreshold', 0.2, 'UniquenessThreshold', 1,...
   'DistanceThreshold', 384);



 subplot(1,2,1), imshow(ILrect, [])
 subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet

end

