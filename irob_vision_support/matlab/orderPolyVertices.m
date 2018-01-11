function [vertices_ordered] = orderPolyVertices(vertices_cw,first_marker)


d = sum(abs(vertices_cw - repmat(first_marker, size(vertices_cw, 1),1)), 2);

[min_d, min_idx] = min(d);

vertices_ordered = vertices_cw;
for i = 1 : (min_idx - 1)
    vertices_ordered = [vertices_ordered(end,:); vertices_ordered(1:end-1,:)];
end
end

