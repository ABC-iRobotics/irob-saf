function disparity_map = correlation_match(left, right, maxdisp, dir)
% disparity_map = correlation_match(left, right, maxdisp): given a pair of stereo
%   @author: Renata Elek
%   images, calculates the disparity (along horizontal direction) subject
%   to a maximum of maxdisp. The direction of matching is specified by dir
%
%   left    : the left image in the stereo pair
%   right   : the right image in the stereo pair
%   maxdisp : maximum disparity accepted
%   dir     : 0 (default) => left-to-right matching, 1 => right-to-left matching
%
%   disparity_map: disparity map- image of the same size as input image
%   but the left (or right) edge is incorrect


if (nargin < 4)
  dir =0;
end

if (nargin < 3)
  % disparity map accuracy:
  maxdisp=15;
end

%filtering with laplacian of gaussian
%lapOfGauss = [0 -1 0 ; -1 4 -1 ; 0 -1 0];
%l1 = conv2(left, lapOfGauss,'same');
%r1 = conv2(right, lapOfGauss,'same');
%figure;imagesc(l1);colormap(gray);axis image;
%left = l1;
%right = r1;


[m n]=size(left);

% 3D array, used to store 0:maxdisp correlation values for each pixel
img = zeros(m,n,1+maxdisp);

% tmp array used to store corr. values
diss  = zeros(m,n);

%window size
windowSize = 5;
window = ones(1, windowSize);

%kernel to sum across the window
corrSumKernel =window'*window; %should probably be made two separate
                               %1-D convs
    
for d=0:maxdisp
 
   % if this value of d doesn't make sense for a column (say d=4 for
   % col#3), it should not be counted. as a precaution set all to Inf
   % the correct columns will get non-Inf values and will win in min()
   diss(:) = Inf;

    if (dir==0) % left-to-right
      
      inds = m*d+1:m*n; %exclude the first d cols

      % SSD metric for correlation calculation
      diss(inds) = (left(inds) - right(inds - m*d)).^2;
      
    else  % right-to-left
      
      inds = 1:m*(n-d); %exclude the last d cols

      % SSD metric for correlation calculation
      diss(inds) = (left(inds + m*d) - right(inds)).^2;
    
    end
      
      
    %update the big matrix
    img(:,:,d+1) = conv2(diss,corrSumKernel,'same');
    
end

[val,ind]=min(img,[],3);
disparity_map = ind-1;

if (nargout ==0) %show output only if the user didn't specify an output
                 %image
  figure;imagesc(ind);colormap(gray);axis image;
end