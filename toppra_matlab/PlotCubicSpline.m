clf;clear;
close all; clc;

rad2deg=180.0/pi;
load('CubicSpline.txt')
% 
T=CubicSpline(:,1);
A1_pos=CubicSpline(:,2);
A1_vel=CubicSpline(:,3);
A1_acc=CubicSpline(:,4);
A2_pos=CubicSpline(:,5);
A2_vel=CubicSpline(:,6);
A2_acc=CubicSpline(:,7);

% 
% 
figure(1)
subplot(3,1,1);
plot(T,A1_pos*rad2deg,'r'); hold on;
plot(T,A2_pos*rad2deg,'b'); 
legend('A1\_pos','A2\_pos');
xlabel('time');
ylabel('pos');
title('pos');
grid on;
grid minor;

subplot(3,1,2);
plot(T,A1_vel*rad2deg,'r');hold on;
plot(T,A2_vel*rad2deg,'b');
legend('A1\_vel','A2\_vel');
xlabel('time');
ylabel('vel');
title('vel');
grid on;
grid minor;

subplot(3,1,3);
plot(T,A1_acc*rad2deg,'r');hold on;
plot(T,A2_acc*rad2deg,'b');
legend('A1\_acc','A1\_acc');
xlabel('time');
ylabel('acc');
title('acc');
grid on;
grid minor;
suptitle('CRC\_A1  VS  CRC\_A2');
