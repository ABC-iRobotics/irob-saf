%% Read rosbag

clear all;
bag = rosbag('2021-01-13-14-34-43_best.bag')
%bag = rosbag('2021-01-13-14-32-50_better.bag')


%% Read specifi topics

crop_start = bag.StartTime + 11.0
crop_end = bag.StartTime + 15.0


bag.AvailableTopics
position_cartesian_bsel = select(bag,'Time',...
    [crop_start crop_end],'Topic','/dvrk/PSM1/position_cartesian_current');

pos_structs = readMessages(position_cartesian_bsel,'DataFormat','struct');

jaw_bsel = select(bag,'Time',...
    [crop_start crop_end],'Topic','dvrk/PSM1/state_jaw_current');

jaw_structs = readMessages(jaw_bsel,'DataFormat','struct');



%% Parse data

p_x = zeros(length(pos_structs),1);
p_y = zeros(length(pos_structs),1);
p_z = zeros(length(pos_structs),1);
t = zeros(length(pos_structs),1);

o_x = zeros(length(pos_structs),1);
o_y = zeros(length(pos_structs),1);
o_z = zeros(length(pos_structs),1);
o_w = zeros(length(pos_structs),1);

j = zeros(length(jaw_structs),1);
j_t = zeros(length(jaw_structs),1);

for i = 1:length(pos_structs)
    p_x(i) = pos_structs{i}.Pose.Position.X;
    p_y(i) = pos_structs{i}.Pose.Position.Y;
    p_z(i) = pos_structs{i}.Pose.Position.Z;
    
    o_x(i) = pos_structs{i}.Pose.Orientation.X;
    o_y(i) = pos_structs{i}.Pose.Orientation.Y;
    o_z(i) = pos_structs{i}.Pose.Orientation.Z;
    o_w(i) = pos_structs{i}.Pose.Orientation.W;
    
    t(i) = seconds(rostime(pos_structs{i}.Header.Stamp.Sec, pos_structs{i}.Header.Stamp.Nsec)) - crop_start;
    
end

for i = 1:length(jaw_structs)
    j(i) = jaw_structs{i}.Position;
    
    j_t(i) = seconds(rostime(jaw_structs{i}.Header.Stamp.Sec, jaw_structs{i}.Header.Stamp.Nsec)) - crop_start;
    
end

%% Plot 3D

plot3(p_x,p_y,p_z)
xlabel('X')
ylabel('Y')
zlabel('Z')

%% Plots

subplot(2,1,1);
plot(t, p_x, 'r', t, p_y, 'g', t, p_z, 'b');

subplot(2,1,2);
plot(j_t, j);

%% Save to file

T = table(t,p_x,p_y,p_z,o_x,o_y,o_z,o_w,j)
writetable(T,'approach_1.dat','Delimiter',' ')



























