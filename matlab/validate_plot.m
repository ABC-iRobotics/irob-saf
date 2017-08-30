err = load('err_papers_meats.mat');
err = err.err;
err_m = zeros(0);
groups = zeros(0);
for i = 1:8
    err_m = [err_m, err{i}];
    groups = [groups, ones(size(err{i})) * i];
end

boxplot(abs(err_m), groups, 'Symbol','w*', 'Labels',{'Plain white','Checkerboard','Rough surface','Kraft paper', 'Dissection phantom', 'Duck liver', 'Chicken breast', 'Pork shoulder'})

title('Dissection line extracion method sensitiveness to texture')
xlabel('Textures')
ylabel('Absolute error  [pix]')