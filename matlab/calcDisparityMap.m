function [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  IL, IR, stereoParams )

maxDisp = 96;

%Rectify the images
[ILrect, IRrect] = ...
rectifyStereoImages(IL, IR, stereoParams.stereoParams);

frameLeftGray  = rgb2gray(ILrect);
frameRightGray = rgb2gray(IRrect);


disparityRange = [0,maxDisp];
disparityMap = disparity(frameLeftGray,frameRightGray,'BlockSize',...
   15,'DisparityRange',disparityRange, 'ContrastThreshold', 0.2, 'UniquenessThreshold', 1,...
   'DistanceThreshold', maxDisp);


row = numel(disparityMap(:,1));
column = numel(disparityMap(1,:));
interpolateVector = reshape(disparityMap,1, row*column);

if interpolateVector(1) < 1
    interpolateVector(1) = 70;
end

A =size(interpolateVector);
A = A(1,2);
for i=2:A
   if or(interpolateVector(i) < 20, interpolateVector(i) > 200)
       interpolateVector(i) = interpolateVector(i-1);
   end
    
end

disparityMapInterpolated = reshape(interpolateVector,row,column);
disparityMap = disparityMapInterpolated;


% subplot(1,2,1), imshow(ILrect, [])
% subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet

end