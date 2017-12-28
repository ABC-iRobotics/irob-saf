function [ disparityMapInterpolated ] = interpolateDisparityMap( disparityMap, zero_value )


disparityMap = disparityMap - zero_value;


row = numel(disparityMap(:,1));
column = numel(disparityMap(1,:));
interpolateVector = reshape(disparityMap,1, row*column);

if interpolateVector(1) < 1
    interpolateVector(1) = 70;
end

A =size(interpolateVector);
A = A(1,2);
for i=2:A
    if or(interpolateVector(i) < 20, interpolateVector(i) > 200)
        interpolateVector(i) = interpolateVector(i-1);
    end
    
end

disparityMapInterpolated = reshape(interpolateVector,row,column) + zero_value;
end

