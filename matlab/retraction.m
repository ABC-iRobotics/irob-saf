clear all;
close all;

IL = imread('le.jpg');
IR = imread('ri.jpg');

calib = load('calibrationSession.mat');
stereoParams = load('stereoParams.mat');
% 
% %Rectify the images
[ILrect, IRrect] = ...
 rectifyStereoImages(IL, IR, stereoParams.stereoParams);

 frameLeftGray  = rgb2gray(ILrect);
 frameRightGray = rgb2gray(IRrect);

disparityRange = [0,400];
disparityMap = disparity(frameLeftGray,frameRightGray,'BlockSize',...
    15,'DisparityRange',disparityRange, 'ContrastThreshold', 0.1, 'UniquenessThreshold', 1,...
   'DistanceThreshold', 384);



figure('units','normalized','outerposition',[0 0 1 1])
subplot(1,2,1), imshow(ILrect, [])
subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
 
[x,y] = ginput(2);
    
    
lowThresholdY = uint32(y(1) - 20);
highThresholdY = uint32(y(1) + 20);

MinimaArrayX = double(zeros(0));
MinimaArrayY = double(zeros(0));
MinimaValues = double(zeros(0));

orientationOverY = double(zeros(0)); %folott
orientationUnderY = double(zeros(0));
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
 highThresh = 20;
 lowThresh = 20;
  for i = 1 : numel(MinimaArrayX)
    orientationOverY = [orientationOverY, MinimaArrayY(i) + (2*highThresh)];
    orientationUnderY = [orientationUnderY, MinimaArrayY(i) - (2*lowThresh)];
    disp(orientationOverY);
    disp(orientationUnderY);
 end

subplot(1,2,2), imshow(ILrect, [])
hold on
plot(MinimaArrayX, MinimaArrayY -7, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
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
%im_coord_R_over = transpose([MinimaArrayX + MinimaValues; orientationOverY ]);

im_coord_L_under = transpose([MinimaArrayX; orientationUnderY ]);
%im_coord_R_under = transpose([MinimaArrayX + MinimaValues; orientationUnderY ]);

  
points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', ILrect);

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

%under
cuttingXYZIdx3 = uint32(round(sub2ind(size(disparityMap), im_coord_L_under(:, 2), im_coord_L_under(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZUnder = [X(cuttingXYZIdx3)'; Y(cuttingXYZIdx3)'; Z(cuttingXYZIdx3)']';
cuttingXYZUnder = cuttingXYZUnder(isfinite(cuttingXYZUnder(:,1)),:);

% Find the distances from the camera in meters.
dists = sqrt(sum(cuttingXYZ' .^ 2));


c = cuttingXYZOver(1,:);
a = cuttingXYZUnder(1,:);
b = cuttingXYZ(1,:);
 
v1 = [a(:,1) - b(:,1), a(:,2) - b(:,2), a(:,3) - b(:,3)];
v2 = [c(:,1) - b(:,1), c(:,2) - b(:,2), c(:,3) - b(:,3)];

angle = acos(dot(v1,v2));
angle = angle * (180/pi);

magicnumber = 200; 
v3 = [(a(:,1) + magicnumber) - (b(:,1)+magicnumber), (a(:,2) + magicnumber) - (b(:,2) + magicnumber), (a(:,3)+magicnumber) - (b(:,3) + magicnumber)];
v4 = [(c(:,1) + magicnumber) - (b(:,1) + magicnumber), (c(:,2) + magicnumber) - (b(:,2) + magicnumber), (c(:,3) + magicnumber) - (b(:,3) + magicnumber)];

angle2 = acos(dot(v3,v4));
angle2 = angle2 * (180/pi);


%dx = gradient((1:size(data)));
dy = gradient(data);
%curv = gradient(atan(dy)) ./ hypot(dy,0);
