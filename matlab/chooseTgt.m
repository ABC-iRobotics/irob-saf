function [ minIdx, lastIdxs ] = chooseTgt(cuttingXYZ, groupN, lastIdxs, memN)

%groupN = 5;

dataZ = double(zeros((idivide(uint32(size(cuttingXYZ,1))-1, groupN, 'floor')), groupN));
 for i = 1:(groupN*idivide(size(cuttingXYZ,1)-1, uint32(groupN), 'floor'))
     dataZ((idivide(i-1, uint32(groupN), 'floor'))+1, mod(i-1,groupN)+1) = cuttingXYZ(i,3);
%      disp(i)
%      disp(dataZ)
 end
 
 avgDataZ = mean(dataZ, 2);
 
 [B, I] = sort(avgDataZ);
 minIdx = 0;
 i = 0;
 while minIdx == 0
     i = i + 1;
     if not(any(I(i) == lastIdxs))
       minIdx = I(i);
     end
 end
 
 if size(lastIdxs) < memN
    lastIdxs  = [lastIdxs, minIdx];
 else
     lastIdxs  = [lastIdxs(2:end), minIdx];
 end
 
 
end

