function [ y_top, y_bottom ] = phantomSegmentation( disparityMap, prev_im_coord_L)

y_top = 310;
y_bottom = 440;

threshSegmentationDiff = 10;

segmentedColumnsIndexesUp = double(zeros(0)); %up
segmentedColumnsIndexesDown = double(zeros(0)); %down


for i = 1:size(prev_im_coord_L, 1) %rows
    
    tempColumnVector = disparityMap(:,uint32(prev_im_coord_L(i,1)));
    sizeDM = size(disparityMap(:,uint32(prev_im_coord_L(i,1))));
    
    for j = uint32(prev_im_coord_L(i,2)): sizeDM(1,1)
        diff = abs(tempColumnVector(j) - tempColumnVector(j-1));
        if diff > threshSegmentationDiff
            segmentedColumnsIndexesDown = [segmentedColumnsIndexesDown, j];
            break
        end
    end
    
    for j = uint32(prev_im_coord_L(i,2)):-1: 2
        %disp('a')
        diff = abs(tempColumnVector(j) - tempColumnVector(j-1));
        if diff > threshSegmentationDiff
            segmentedColumnsIndexesUp = [segmentedColumnsIndexesUp, j];
            break
        end
    end
    
end

prev_mean_y = mean(prev_im_coord_L(:,2));

y_top = mode(segmentedColumnsIndexesUp)
y_bottom = mode(segmentedColumnsIndexesDown)

if (y_bottom - prev_mean_y) > (prev_mean_y - y_top)
    y_bottom = prev_mean_y + (prev_mean_y - y_top)
end

height = size(disparityMap, 1);
if isnan(y_bottom)
     y_bottom = 450;
end

if isnan(y_top)
     y_top = 300;
end

if (y_top < 1)
    y_top = 1;
end

if (y_top > height)
    y_top = height;
end

if (y_bottom < 1)
    y_bottom = 1;
end

if (y_bottom > height)
    y_bottom = height;
end


    


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

