function [ tgt_pos, tgt_ori ] = getTgt(tgt_idx, cuttingXYZ, cuttingXYZOver, cuttingXYZUnder )

tgt_pos = cuttingXYZ(tgt_idx, :);

vec_over = cuttingXYZ(tgt_idx, :) - cuttingXYZOver(tgt_idx, :);
vec_under = cuttingXYZ(tgt_idx, :) - cuttingXYZUnder(tgt_idx, :);

%vec_over(1) = 0.0;
%vec_under(1) = 0.0;

vec_over = vec_over/norm(vec_over)
vec_under = vec_under/norm(vec_under)

w_over = 1;
w_under = 0;

ori_vec = ((vec_over*w_over) + (vec_under*w_under)) / (w_over+w_under);

base_vec = [0.0 0.0 1.0];

tgt_R = vrrotvec2mat(vrrotvec(base_vec, ori_vec))

R = rotz(0.0);

tgt_ori = rotm2quat(tgt_R)

end

