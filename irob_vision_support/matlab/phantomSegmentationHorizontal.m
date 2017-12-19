function [ x_left, x_right ] = phantomSegmentationHorizontal( disparityMap, prev_im_coord_L)

% x_left = 310;
% x_right = 440;
% 
% threshSegmentationDiff = 10;
% 
% segmentedRowssIndexesUp = double(zeros(0)); %up
% segmentedRowsIndexesDown = double(zeros(0)); %down
% 
% 
% for i = 1:size(prev_im_coord_L, 1) %rows
%     
%     tempRowVector = disparityMap(uint32(prev_im_coord_L(i,2)),:);
%     sizeDM = size(disparityMap(uint32(prev_im_coord_L(i,2)),:));
%     
%     for j = uint32(prev_im_coord_L(i,1)): sizeDM(1,1)
%         diff = abs(tempRowVector(j) - tempRowVector(j-1));
%         if diff > threshSegmentationDiff
%             segmentedRowsIndexesDown = [segmentedRowsIndexesDown, j];
%             break
%         end
%     end
%     
%     for j = uint32(prev_im_coord_L(i,1)):-1: 2
%         %disp('a')
%         diff = abs(tempRowVector(j) - tempRowVector(j-1));
%         if diff > threshSegmentationDiff
%             segmentedRowssIndexesUp = [segmentedRowssIndexesUp, j];
%             break
%         end
%     end
%     
% end
% 
% prev_mean_x = mean(prev_im_coord_L(:,1));
% 
% x_left = mode(segmentedRowssIndexesUp);
% x_right = mode(segmentedRowsIndexesDown);
% 
% if (x_right - prev_mean_x) > (prev_mean_x - x_left)
%     x_right = prev_mean_x + (prev_mean_x - x_left);
% end
% 
% height = size(disparityMap, 1);
% if isnan(x_right)
%      x_right = 450;
% end
% 
% if isnan(x_left)
%      x_left = 300;
% end
% if (x_left < 1)
%     x_left = 1;
% end
% 
% if (x_left > height)
%     x_left = height;
% end
% 
% if (x_right < 1)
%     x_right = 1;
% end
% 
% if (x_right > height)
%     x_right = height;
% end

x_right = 480;
x_left = 245;

%imshow(disparityMap, [64,192]);

    
%
% if (y_bottom - prev_mean_y) > 30
%     y_bottom = prev_mean_y + 30
% end
%
%
% if (prev_mean_y - y_top) > 30
%     y_top = prev_mean_y - 30
% end

%subplot(1,2,2), plot(prev_im_coord_L(i, 1),segmentedColumnsIndexesUp);
%segmentedColumnsIndexesUp

end

