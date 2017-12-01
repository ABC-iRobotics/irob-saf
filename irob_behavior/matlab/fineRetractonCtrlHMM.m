%function [ y, z, done ] = fineRetractonCtrlHMM( angle, tension, visible_size )

N = 19;
N_E = 190;
start_angle = 0;
end_angle = 180;
transition_slope = 40;
transition_max_d = 30;

emission_sigma = 30;

STATES = start_angle:((end_angle - start_angle) / (N-1)):end_angle;

T = double(zeros(N,N));

for i = 1:N
    a1 = STATES(i) - (transition_max_d + transition_slope);
    b1 = STATES(i) - transition_max_d;
    c1 = STATES(i) - (transition_max_d - transition_slope);
    
    a2 = STATES(i) + (transition_max_d - transition_slope);
    b2 = STATES(i) + transition_max_d;
    c2 = STATES(i) + (transition_max_d + transition_slope);
    
    
    y = trimf(STATES,[a1 b1 c1]) + trimf(STATES,[a2 b2 c2]);
    
    %T(i,:) = y / sum(y);
    T(i,:) = y / 2;
    %plot(STATES,T(i,:)); hold on;
end

EMISSIONS = start_angle:((end_angle - start_angle) / (N_E-1)):end_angle;

E = double(zeros(N,N_E));

for i = 1:N
    
    E(i, :) = gaussmf(EMISSIONS,[emission_sigma STATES(i)]);
    %plot(EMISSIONS,E(i,:)); hold on;
end

seq = 1:30:190;

likelystates = hmmviterbi(seq, T, E);

plot(likelystates)

%y = output(1)
%z = output(2)



