function [im_coord,corners, found] = getMarkerCoordinates(markers_msg,id)

corners = zeros(4, 2);
found = false;

for i = 1:size(markers_msg.Markers)
    if markers_msg.Markers(i).Id == id
        found = true;
        for j = 1:size(markers_msg.Markers(i).Corners)
            corners(j, 1) = markers_msg.Markers(i).Corners(j).X;
            corners(j, 2) = markers_msg.Markers(i).Corners(j).Y;
        end
    end
end

im_coord =  mean(corners, 1);


end

