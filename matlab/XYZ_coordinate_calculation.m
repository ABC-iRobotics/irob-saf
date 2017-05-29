function [cuttingXYZ, userInputX, userInputY, meanAnglesCenter, meanAnglesTop] = XYZ_coordinate_calculation( IL, IR, stereoParams, maxDisp, dir, lowThresh, highThresh, firstTgt, userInputX, userInputY)
%UNTITLED2 Summary of this function goes here
%   @author: Renata Elek
%   IL & IR: left and right images of your stereo cameras (what are you
%   working on)
%   calibrationSession: the saved session of your stereo calibration
%   for example: 'calibrationSessionFinal.mat'
%   stereoParams: exported parameters of your stereo camera
%   for example: 'stereoParamsFinal.mat'
%   maxDisp : parameter of the disparity map function; maximum disparity
%   accepted (500 works well)
%   dir: parameter of the disparity map function; 0 (default) => left-to-right matching,
%   1 => right-to-left matching; use 0 (DO NOT TRUST IN DEFAULT)
%   lowThresh, highThresh: thresholds in y coordinates to find peaks (20
%   was the best)

%calib = load(calibrationSession);


%Rectify the images
[ILrect, IRrect] = ...
rectifyStereoImages(IL, IR, stereoParams.stereoParams);

frameLeftGray  = rgb2gray(ILrect);
frameRightGray = rgb2gray(IRrect);

% frameLeftGray  = ILrect(:,:,2);
%  frameRightGray = IRrect(:,:,2);
% % 
%    H = fspecial('gaussian',[3 3],0.5);
%  frameLeftGray = imfilter(frameLeftGray,H);
%    frameRightGray = imfilter(frameRightGray,H);



disparityRange = [0,400];
disparityMap = disparity(frameLeftGray,frameRightGray,'BlockSize',...
   15,'DisparityRange',disparityRange, 'ContrastThreshold', 0.2, 'UniquenessThreshold', 1,...
   'DistanceThreshold', 384);

% se = strel('disk',3);
% disparityMap = imclose(disparityMap, se);

disparityRange = [0,maxDisp];
% 
% figure('units','normalized','outerposition',[0 0 1 1])
 subplot(1,2,1), imshow(ILrect, [])
 subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
 
if firstTgt 
     [x,y] = ginput(2);
    userInputX = x;     userInputY = y;
%     x = [39.529147982062796;2.363482810164425e+02]
%     y = [4.045209267563528e+02;3.963953662182362e+02]
%     userInputX =[39.529147982062796;2.363482810164425e+02]
%     userInputY = [4.045209267563528e+02;3.963953662182362e+02]
else
    x = userInputX;
    y = userInputY;
end
    
lowThresholdY = uint32(y(1) - 20);
highThresholdY = uint32(y(1) + 20);

MinimaArrayX = double(zeros(0));
MinimaArrayY = double(zeros(0));
MinimaValues = double(zeros(0));

orientationOverY = double(zeros(0)); %folott
orientationUnderY = double(zeros(0));
orientationOverYTop = double(zeros(0)); %folott
orientationUnderYTop = double(zeros(0));

%figure
dataM = double(zeros(0));
 for i = uint32(x(1)) : uint32(x(2))
    data = disparityMap(lowThresholdY: highThresholdY, i); 
    data = double(data);
    data = smooth(data, 'moving');
    subplot(1,2,1), plot(data)
hold on
    dataM = cat(2,dataM, data);
    [Minima,MinIdx] = findpeaks(-data, 'Npeaks', 1);
    
    if  isfinite(MinIdx)
        
        MinimaArrayX = [MinimaArrayX, double(i)];
        MinimaArrayY = [MinimaArrayY, double(lowThresholdY + MinIdx)];
        MinimaValues = [MinimaValues, double(Minima)];
    
    else 
        disp('The value was inf.');
        MinimaArrayX = [MinimaArrayX, double(i)];
        MinimaArrayY = [MinimaArrayY, double(y(1))];
        MinimaValues = [MinimaValues, -disparityMap(uint32(y(1)),uint32(x(1)))];
        %MinimaValues = [MinimaValues, 1.0];
    end
 end
%     title({'Plot of vertical';'disparity changes in the ROI'})
%     xlabel('Pixel indices in vertical direction')
%     ylabel('Disparity value [px]')
%     axis tight
 highThresh = 8;
 lowThresh = 8;
  for i = 1 : numel(MinimaArrayX)
    orientationOverYTop = [orientationOverYTop, MinimaArrayY(i) - (4*highThresh)];
    orientationUnderYTop = [orientationUnderYTop, MinimaArrayY(i) - (2*lowThresh)];
    
    orientationOverY = [orientationOverY, MinimaArrayY(i) + (4*highThresh)];
    orientationUnderY = [orientationUnderY, MinimaArrayY(i) - (4*lowThresh)];
 end

subplot(1,2,2), imshow(ILrect, [])
hold on
plot(MinimaArrayX, MinimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
hold off

subplot(1,3,1), imshow(disparityMap,disparityRange);colormap jet
 hold on

plot(MinimaArrayX, MinimaArrayY,MinimaArrayX, orientationOverY, MinimaArrayX, orientationUnderY,  'r.');
hold off

%surf(dataM);

% 
%  
% imshow(ILrect);
% hold on
% plot(MinimaArrayX, MinimaArrayY, 'r.');
  
im_coord_L = transpose([MinimaArrayX; MinimaArrayY ]);
im_coord_R = transpose([MinimaArrayX + MinimaValues; MinimaArrayY ]);

im_coord_L_over = transpose([MinimaArrayX; orientationOverY ]);
im_coord_L_overTop = transpose([MinimaArrayX; orientationOverYTop ]);
%im_coord_R_over = transpose([MinimaArrayX + MinimaValues; orientationOverY ]);

im_coord_L_under = transpose([MinimaArrayX; orientationUnderY ]);
im_coord_L_underTop = transpose([MinimaArrayX; orientationUnderYTop ]);
%im_coord_R_under = transpose([MinimaArrayX + MinimaValues; orientationUnderY ]);

  
points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
points3D = points3D ./ 1000;
%ptCloud = pointCloud(points3D, 'Color', ILrect);

cuttingXYZIdx1 = uint32(round(sub2ind(size(disparityMap), im_coord_L(:, 2), im_coord_L(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZ = [X(cuttingXYZIdx1)'; Y(cuttingXYZIdx1)'; Z(cuttingXYZIdx1)']';
cuttingXYZ = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);

%over
cuttingXYZIdx2 = uint32(round(sub2ind(size(disparityMap), im_coord_L_over(:, 2), im_coord_L_over(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZOver = [X(cuttingXYZIdx2)'; Y(cuttingXYZIdx2)'; Z(cuttingXYZIdx2)']';
cuttingXYZOver = cuttingXYZOver(isfinite(cuttingXYZOver(:,1)),:);

%over
cuttingXYZIdx4 = uint32(round(sub2ind(size(disparityMap), im_coord_L_overTop(:, 2), im_coord_L_overTop(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZOverTop = [X(cuttingXYZIdx4)'; Y(cuttingXYZIdx4)'; Z(cuttingXYZIdx4)']';
cuttingXYZOverTop = cuttingXYZOverTop(isfinite(cuttingXYZOverTop(:,1)),:);


%under
cuttingXYZIdx3 = uint32(round(sub2ind(size(disparityMap), im_coord_L_under(:, 2), im_coord_L_under(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZUnder = [X(cuttingXYZIdx3)'; Y(cuttingXYZIdx3)'; Z(cuttingXYZIdx3)']';
cuttingXYZUnder = cuttingXYZUnder(isfinite(cuttingXYZUnder(:,1)),:);

cuttingXYZIdx5 = uint32(round(sub2ind(size(disparityMap), im_coord_L_underTop(:, 2), im_coord_L_underTop(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZUnderTop = [X(cuttingXYZIdx5)'; Y(cuttingXYZIdx5)'; Z(cuttingXYZIdx5)']';
cuttingXYZUnderTop = cuttingXYZUnderTop(isfinite(cuttingXYZUnderTop(:,1)),:);

% Find the distances from the camera in meters.
dists = sqrt(sum(cuttingXYZ' .^ 2));
disp('size');
disp(size(cuttingXYZ(:,1)));
anglesArray = double(zeros(0));
for i = 1:1:min([size(cuttingXYZ(:,1)),size(cuttingXYZUnder(:,1)),size(cuttingXYZOver(:,1))])
    c = cuttingXYZOver(i,:);
    a = cuttingXYZUnder(i,:);
    b = cuttingXYZ(i,:);
 
    v1 = [a(:,1) - b(:,1), a(:,2) - b(:,2), a(:,3) - b(:,3)];
    v2 = [c(:,1) - b(:,1), c(:,2) - b(:,2), c(:,3) - b(:,3)];
    v3 = [a(:,1) - c(:,1), a(:,2) - c(:,2), a(:,3) - c(:,3)];

    v1mag = sqrt(abs(v1(1) * v1(1) + v1(2) * v1(2) + v1(3) * v1(3)));
    v2mag = sqrt(abs(v2(1) * v2(1) + v2(2) * v2(2) + v2(3) * v2(3)));
    v3mag = sqrt(abs(v3(1) * v3(1) + v3(2) * v3(2) + v3(3) * v3(3)));

    angle = acos(((v2mag * v2mag) + (v1mag * v1mag) - (v3mag * v3mag))/ (2*v2mag*v1mag));
    angle = angle * (180/pi);
    anglesArray = [anglesArray, angle];
end

meanAnglesCenter = mean(anglesArray);

anglesArrayTop = double(zeros(0));

for i = 1:min([size(cuttingXYZ(:,1)),size(cuttingXYZOverTop(:,1)),size(cuttingXYZUnderTop(:,1))])
    c = cuttingXYZOverTop(i,:);
    b = cuttingXYZUnderTop(i,:);
    a = cuttingXYZ(i,:);
 
    v1 = [a(:,1) - b(:,1), a(:,2) - b(:,2), a(:,3) - b(:,3)];
    v2 = [c(:,1) - b(:,1), c(:,2) - b(:,2), c(:,3) - b(:,3)];
    v3 = [a(:,1) - c(:,1), a(:,2) - c(:,2), a(:,3) - c(:,3)];

    v1mag = sqrt(abs(v1(1) * v1(1) + v1(2) * v1(2) + v1(3) * v1(3)));
    v2mag = sqrt(abs(v2(1) * v2(1) + v2(2) * v2(2) + v2(3) * v2(3)));
    v3mag = sqrt(abs(v3(1) * v3(1) + v3(2) * v3(2) + v3(3) * v3(3)));

    angle = acos(((v2mag * v2mag) + (v1mag * v1mag) - (v3mag * v3mag))/ (2*v2mag*v1mag));
    angle = angle * (180/pi);
    anglesArrayTop = [anglesArrayTop, angle];
end
meanAnglesTop = mean(anglesArrayTop);

end