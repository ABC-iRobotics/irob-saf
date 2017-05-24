function [ tgt_pos, tgt_ori ] = getTgt(tgt_idx,  groupMinIdx, groupN, cuttingXYZ)

tgt_pos = cuttingXYZ(((groupMinIdx-1) * groupN) + tgt_idx, :);
tgt_pos = [0.1 0.0 0.4];

% 
% vec_over = mean(cuttingXYZ, 1) - mean(cuttingXYZOver, 1);
% vec_under = mean(cuttingXYZ,1) - mean(cuttingXYZUnder,1);
% 
% %vec_over(1) = 0.0;
% %vec_under(1) = 0.0;
% 
% vec_over = vec_over/norm(vec_over)
% vec_under = vec_under/norm(vec_under)
% 
% w_over = 1;
% w_under = 0;
% 
% ori_vec = ((vec_over*w_over) + (vec_under*w_under)) / (w_over+w_under);
% 
% base_vec = [0.0 0.0 1.0];
% 
% tgt_R = vrrotvec2mat(vrrotvec(base_vec, ori_vec))
% 
% R = rotz(0.0);

%tgt_ori = rotm2quat(tgt_R)
tgt_ori = zeros(4,1);
end

