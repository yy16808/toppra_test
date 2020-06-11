clf;clear;close all; clc;


load('CubicSpline.txt')
% 
T=CubicSpline(:,1);
A1_pos=CubicSpline(:,2);
A1_vel=CubicSpline(:,3);
A1_acc=CubicSpline(:,4);

% 
% 
figure(1)
subplot(3,1,1);
plot(T,A1_pos,'r');
legend('vel');
xlabel('time');
ylabel('pos');
title('motor1\_pos');
grid on;
grid minor;

subplot(3,1,2);
plot(T,A1_vel,'g');
legend('vel');
xlabel('time');
ylabel('vel');
title('motor1\_vel');
grid on;
grid minor;

subplot(3,1,3);
plot(T,A1_acc,'b');
legend('acc');
xlabel('time');
ylabel('acc');
title('motor1\_acc');
grid on;
grid minor;
