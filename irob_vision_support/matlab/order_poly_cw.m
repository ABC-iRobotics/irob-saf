function [ordered_vertices] = order_poly_cw(vertices)

% Found this snipplet on StackOwerflow

x = vertices(:,1);
y = vertices(:,2);

cx = mean(x);
cy = mean(y);

% Find the angles:

a = atan2(y - cy, x - cx);

% Find the correct sorted order:

[~, order] = sort(a);

% Reorder the coordinates:

x = x(order);
y = y(order);

ordered_vertices = [x, y];

end

