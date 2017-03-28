function [ minIdx ] = chooseTgt(cuttingXYZ, groupN, lastIdx)

%groupN = 5;

dataZ = double(zeros((idivide(uint32(size(cuttingXYZ,1)), groupN, 'floor')), groupN));
 for i = 1:(groupN*idivide(size(cuttingXYZ,1), uint32(groupN), 'floor'))
     dataZ((idivide(i-1, uint32(groupN), 'floor'))+1, mod(i-1,groupN)+1) = cuttingXYZ(i,3);
%      disp(i)
%      disp(dataZ)
 end
 
 avgDataZ = mean(dataZ, 2);
 
 [B, I] = sort(avgDataZ);
 if not(I(1) == lastIdx)
       minIdx = I(1);
 else
       minIdx = I(2);
 end

end

