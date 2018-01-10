function [vertices_ordered] = orderPolyVertices(vertices_cw,first_marker)


d = vertices_cw - repmat(first_marker, size(vertices_cw, 1),1);

[min_d, min_idx] = min(d);

vertices_ordered = [vertices_cw(min_idx:end,:); vertices_cw(1:min_idx-1)];

end

