fileID = fopen('trajectory_1.dat','r');
A = fscanf(fileID,'%f%f%f%f%f%f%f%f\n');
fclose(fileID);

A = reshape(A, 8, [] )';

%disp(A);


plot3(A(:,1), A(:,2), A(:,3))
