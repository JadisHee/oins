close all;
clear;
clc;

data = xlsread('./record_3.csv');

state = data(1:1500,1);

encoder_speed = data(1:1500,2);

ins_velocity = data(1:1500,3);

slam_displacement = data(1:1500,4);

ins_displacement = data(1:1500,5);

error_k = zeros(length(slam_displacement),1);
for i = 1:1:length(slam_displacement)
    error_k(i) = abs(slam_displacement(i) - ins_displacement(i));
end

figure(1)
plot(encoder_speed);
hold on 
plot(ins_velocity);
legend('编码器速度','惯导速度')

figure(2)
plot(slam_displacement);
hold on 
plot(ins_displacement);
hold on 
plot(error_k);
legend('slam位移','惯导位移','误差')
