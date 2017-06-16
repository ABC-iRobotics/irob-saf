function [ minimaArrayX, minimaArrayY, minimaValues ] = findDisparityMinimas(disparityMap, prev_im_coord_L, top_margin, bottom_margin )

    minimaArrayX = double(zeros(0));
    minimaArrayY = double(zeros(0));
    minimaValues = double(zeros(0));

    dataM = double(zeros(0));
    
%     disp('top_margin');
%     disp(top_margin);
%     
%     disp('bottom_margin');
%     disp(bottom_margin);
%     
%     disp('prev_im_coord');
%     disp(prev_im_coord_L(:,2));
%     
%     disp('disp map size');
%     disp(size(disparityMap));
    
    for i = 1:size(prev_im_coord_L, 1)
        %disp(i);
        %disp(prev_im_coord_L(i,2));
        %disp(prev_im_coord_L(i,2));
        %disp((prev_im_coord_L(i,2)-top_margin): (prev_im_coord_L(i,2)+bottom_margin));
        data = disparityMap((uint32(prev_im_coord_L(i,2)-top_margin)): uint32((prev_im_coord_L(i,2)+bottom_margin)), uint32(prev_im_coord_L(i,1))); 
        data = double(data);
        data = smooth(data, 'moving');
        subplot(1,2,1), plot(data)
        hold on
        dataM = cat(2,dataM, data);
        [Minima,MinIdx] = findpeaks(-data, 'Npeaks', 1);
    
        if  isfinite(MinIdx)
        
            minimaArrayX = [minimaArrayX, double(prev_im_coord_L(i,1))];
            minimaArrayY = [minimaArrayY, double((prev_im_coord_L(i,2)-top_margin) + MinIdx)];
            minimaValues = [minimaValues, double(Minima)];
    
        else 
            disp('The value was inf.');
            minimaArrayX = [minimaArrayX, double(prev_im_coord_L(i,1))];
            minimaArrayY = [minimaArrayY, double((prev_im_coord_L(i,2)-top_margin))];
            %minimaArrayY = [minimaArrayY, double(prev_im_coord_L(i,2))];
            minimaValues = [minimaValues, -disparityMap(prev_im_coord_L(i,2), prev_im_coord_L(i,1))];
        end
    end
    %     title({'Plot of vertical';'disparity changes in the ROI'})
    %     xlabel('Pixel indices in vertical direction')
    %     ylabel('Disparity value [px]')
    %     axis tight

    minimaArrayY = hampel(minimaArrayY);

end

