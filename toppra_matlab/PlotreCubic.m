clf;clear;close all; clc;
deg2rad=pi/180;
rad2deg=180/pi;
 P1 = 1.0602, P2 = 0.2433, P3 = 0.2438, Jm1 = 0.6496;

 mp=1;%payload=1
L1=0.225;L2=0.275;
 P1 = 1.0602+mp*L1^2 , P2 = 0.2433+mp*L2^2 , P3 = 0.2438, Jm1 = 0.6496;

 q1=-132*deg2rad;
 q2=-79*deg2rad;
 h1=P1-Jm1+P2+2*cos(q2)*P3;
 h2=P2+cos(q2)*P3;
 m1=P2+cos(q2)*P3;
 m2=P2;
 A=[h1,m1;h2,m2];
 Q=[q1,q2]
 dQ=[14*deg2rad,44*deg2rad]';
 A*dQ
 p1_theta1=-132*deg2rad; p1_theta2=-79*deg2rad;
  p2_theta1=-118*deg2rad; p2_theta2=-35*deg2rad;
 L1=0.225;L2=0.275;
p1_x=cos(p1_theta1+p1_theta2)*L2+cos(p1_theta1)*L1;
p1_y=sin(p1_theta1+p1_theta2)*L2+sin(p1_theta1)*L1;

p2_x=cos(p2_theta1+p2_theta2)*L2+cos(p2_theta1)*L1;
p2_y=sin(p2_theta1+p2_theta2)*L2+sin(p2_theta1)*L1;

dis_pt=sqrt((p1_x-p2_x)^2+(p1_y-p2_y)^2);
% P1+P2+2*P3*cos(q2)
% P2+P3*cos(q2)



load('reCubic.txt');

T=reCubic(:,1);
A1_pos=reCubic(:,2);
A1_vel=reCubic(:,3);
A1_acc=reCubic(:,4);
A2_pos=reCubic(:,5);
A2_vel=reCubic(:,6);
A2_acc=reCubic(:,7);

% A1_pos=Test(:,7);
% A2_pos=Test(:,10);
% A1_vel=Test(:,6);
% A2_vel=Test(:,9);
% A1_acc=Test(:,5);
% A2_acc=Test(:,8);
% T=Test(:,4);

%  P1 = 1.0602, P2 = 0.2433, P3 = 0.2438, Jm1 = 0.6496;

ddq1 = A1_acc;
ddq2 = A2_acc;
dq1 = A1_vel;
dq2 = A2_vel;
q1 = A1_pos;
q2 = A2_pos;
for i=1:size(reCubic(:,1))
    tau1(i) = (P1 - Jm1)*ddq1(i) + P2*(ddq1(i) + ddq2(i)) + P3*((2 * ddq1(i) + ddq2(i))*cos(q2(i)) - (dq2(i)*dq2(i) + 2 * dq1(i)*dq2(i))*sin(q2(i)));
    tau2(i) = P2*(ddq1(i) + ddq2(i)) + P3*(ddq1(i)*cos(q2(i))+dq1(i)*dq1(i)*sin(q2(i)));
end


figure(1)
subplot(4,1,1);
plot(T,A1_pos*rad2deg,'r');hold on;
plot(T,A2_pos*rad2deg,'b');
legend('motor1\_pos','motor2\_pos');
xlabel('time');
ylabel('pos');
title('pos');
grid on;
grid minor;

subplot(4,1,2);
plot(T,A1_vel*rad2deg,'r');hold on;
plot(T,A2_vel*rad2deg,'b','Marker','*');
legend('motor1\_vel','motor2\_vel');
xlabel('time');
ylabel('vel');
title('vel');
grid on;
grid minor;

subplot(4,1,3);
plot(T,A1_acc*rad2deg,'r');hold on;
plot(T,A2_acc*rad2deg,'b','Marker','*');
legend('motor1\_acc','motor2\_acc');
xlabel('time');
ylabel('acc');
title('acc');
grid on;
grid minor;

subplot(4,1,4);
plot(T,tau1,'r');hold on;
plot(T,tau2,'b');
legend('motor1\_trq','motor2\_trq');
xlabel('time');
ylabel('trq');
title('trq');
grid on;
grid minor;
suptitle('ÑùÌõ²åÖµ£ºCRC\_A1  VS  CRC\_A2');
% 
% clf;clear; close all; clc;
% 
% rad2deg=180.0/pi;
% load('reCubic.txt');
% % 
% T=reCubic(:,1);
% A1_pos=reCubic(:,2);
% A1_vel=reCubic(:,3);
% A1_acc=reCubic(:,4);
% A2_pos=reCubic(:,5);
% A2_vel=reCubic(:,6);
% A2_acc=reCubic(:,7);
% 
% % 
% % 
% figure(1)
% subplot(3,1,1);
% plot(T,A1_pos*rad2deg,'r'); hold on;
% plot(T,A2_pos*rad2deg,'b'); 
% legend('A1\_pos','A2\_pos');
% xlabel('time');
% ylabel('pos');
% title('pos');
% grid on;
% grid minor;
% 
% subplot(3,1,2);
% plot(T,A1_vel*rad2deg,'r');hold on;
% plot(T,A2_vel*rad2deg,'b');
% legend('A1\_vel','A2\_vel');
% xlabel('time');
% ylabel('vel');
% title('vel');
% grid on;
% grid minor;
% 
% subplot(3,1,3);
% plot(T,A1_acc*rad2deg,'r');hold on;
% plot(T,A2_acc*rad2deg,'b');
% legend('A1\_acc','A1\_acc');
% xlabel('time');
% ylabel('acc');
% title('acc');
% grid on;
% grid minor;
% suptitle('CRC\_A1  VS  CRC\_A2');
