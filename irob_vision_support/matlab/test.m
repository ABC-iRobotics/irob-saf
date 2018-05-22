R = orth(rand(3,3)); % random rotation matrix

if det(R) < 0
    V(:,3) =V(:,3) * ( -1);
    R = V*U';
end

t = rand(3,1); % random translation

n = 4; % number of points
A = rand(n,3);
B = R*A' + repmat(t, 1, n);
B = B';

[ret_R, ret_t] = rigid_transform_3D(A, B);

A2 = (ret_R*A') + repmat(ret_t, 1, n)
A2 = A2'

% Find the error
err = A2 - B;
err = err .* err;
err = sum(err(:));
rmse = sqrt(err/n);

disp(sprintf("RMSE: %f", rmse));
disp("If RMSE is near zero, the function is correct!");
