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



load('Test.txt')

% A1_pos=14*Test(:,3)-132;
% A2_pos=44*Test(:,3)-79;
% A1_vel=14*Test(:,2)*deg2rad;
% A2_vel=44*Test(:,2)*deg2rad;
% A1_acc=14*Test(:,1)*deg2rad;
% A2_acc=44*Test(:,1)*deg2rad;
A1_pos=Test(:,7);
A2_pos=Test(:,10);
A1_vel=Test(:,6);
A2_vel=Test(:,9);
A1_acc=Test(:,5);
A2_acc=Test(:,8);
T=Test(:,4);

%  P1 = 1.0602, P2 = 0.2433, P3 = 0.2438, Jm1 = 0.6496;

ddq1 = A1_acc;
ddq2 = A2_acc;
dq1 = A1_vel;
dq2 = A2_vel;
q1 = A1_pos;
q2 = A2_pos;
for i=1:size(Test(:,1))
    tau1(i) = (P1 - Jm1)*ddq1(i) + P2*(ddq1(i) + ddq2(i)) + P3*((2 * ddq1(i) + ddq2(i))*cos(q2(i)) - (dq2(i)*dq2(i) + 2 * dq1(i)*dq2(i))*sin(q2(i)));
    tau2(i) = P2*(ddq1(i) + ddq2(i)) + P3*(ddq1(i)*cos(q2(i))+dq1(i)*dq1(i)*sin(q2(i)));
end


figure(1)
subplot(3,1,1);
plot(T,A1_pos*rad2deg,'r');hold on;
plot(T,A2_pos*rad2deg,'b');
legend('motor1\_pos','motor2\_pos');
xlabel('time');
ylabel('pos');
title('pos');
grid on;
grid minor;

subplot(3,1,2);
plot(T,A1_vel*rad2deg,'r');hold on;
plot(T,A2_vel*rad2deg,'b','Marker','*');
legend('motor1\_vel','motor2\_vel');
xlabel('time');
ylabel('vel');
title('vel');
grid on;
grid minor;

subplot(3,1,3);
plot(T,A1_acc*rad2deg,'r');hold on;
plot(T,A2_acc*rad2deg,'b');
legend('motor1\_acc','motor2\_acc');
xlabel('time');
ylabel('acc');
title('acc');
grid on;
grid minor;
suptitle('CRC\_A1  VS  CRC\_A2');


limitMax_pos=T*0+55;
limitMax_neg=T*0-55;
limit_pos=T*0+47;
limit_neg=T*0-47;

figure(2)
subplot(2,1,1);
plot(T,tau1,'b','LineWidth',2);
legend('motor1\_gear trq')
xlabel('time');
ylabel('gear trq');
title('gear trq');
grid on;
grid minor;

subplot(2,1,2);
plot(T,tau2,'b','LineWidth',2); hold on;
plot(T,limit_pos,'g');
hold on;
plot(T,limit_neg,'g');
hold on;
plot(T,limitMax_pos,'m');
hold on;
plot(T,limitMax_neg,'m');
legend('motor2\_gear trq','trqlimit=47N','trqlimit=-47N','trqlimitMax=55N','trqlimitMax=-55N')

xlabel('time');
ylabel('gear trq');
title('gear trq');
grid on;
grid minor;

suptitle('CRC\_A1  VS  CRC\_A2');

% 
% figure(1)
% subplot(4,2,1);
% plot(T,A1_pos*rad2deg,'r');
% legend('vel');
% xlabel('time');
% ylabel('pos');
% title('motor1\_pos');
% grid on;
% grid minor;
% 
% subplot(4,2,3);
% plot(T,A1_vel*rad2deg,'g');
% legend('vel');
% xlabel('time');
% ylabel('vel');
% title('motor1\_vel');
% grid on;
% grid minor;
% 
% subplot(4,2,5);
% plot(T,A1_acc*rad2deg,'b');
% legend('acc');
% xlabel('time');
% ylabel('acc');
% title('motor1\_acc');
% grid on;
% grid minor;
% 
% subplot(4,2,7);
% plot(T,tau1,'b');
% legend('trq');
% xlabel('time');
% ylabel('trq');
% title('motor1\_trq');
% grid on;
% grid minor;
% 
% subplot(4,2,2);
% plot(T,A2_pos*rad2deg,'r');
% legend('vel');
% xlabel('time');
% ylabel('pos');
% title('motor2\_pos');
% grid on;
% grid minor;
% 
% subplot(4,2,4);
% plot(T,A2_vel*rad2deg,'g');
% legend('vel');
% xlabel('time');
% ylabel('vel');
% title('motor2\_vel');
% grid on;
% grid minor;
% 
% 
% subplot(4,2,6);
% plot(T,A2_acc*rad2deg,'b');
% legend('acc');
% xlabel('time');
% ylabel('acc');
% title('motor2\_acc');
% grid on;
% grid minor;
% 
% subplot(4,2,8);
% plot(T,tau2,'b');
% legend('trq');
% xlabel('time');
% ylabel('trq');
% title('motor2\_trq');
% grid on;
% grid minor;
