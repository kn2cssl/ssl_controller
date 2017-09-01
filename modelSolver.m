clear all
clc
clf, echo on
tspan=[0 32.7];
x0=[0;0;0;0;0;0;0;0;0;0;0;0;0;0];
global init_flag ;
init_flag = 1;
global real_data ;
global real_data2;
global r_d_size ;
%RLS==========
global p;
global q;
global theta;
theta = zeros(11,7);
q=zeros(11,7);
p=eye(11)*1000 ;
%;;;;;;;;;;;;;

real_data = importdata('ssl-d-13960608-5-forRLS.txt');
r_d_size =  size(real_data);
real_data = [real_data(:,1:8)/1000, real_data(:,9:12), real_data(:,13:16)/1000];

real_data2 = importdata('ssl-d-13960608-4-7ss-2.txt');
r_d_size2 =  size(real_data2);
real_data2 = [real_data2(:,1:8)/1000, real_data2(:,9:12), real_data2(:,13:16)/1000];

[t,x] = ode45(@dynamicModel,tspan, x0, odeset('OutputFcn','odeplot','MaxStep',20*1e-1)); 

%=Robot's speed============================================================
close all
figure;
subplot(2,2,1);
hold all
plot(real_data(:,1),real_data(:,6),':b','LineWidth',2)
plot(t,x(:,1),'r','LineWidth',2)
plot(t,x(:,8),'--g','LineWidth',2)
legend('V_x (real)','V_x  (kalman)','V_x  (setpoint)');
xlabel('Time [sec]');
ylabel('Speed [m/s]');
title('LTR Test')
title('Feed Test')

% figure;
subplot(2,2,2);
hold all
plot(real_data(:,1),real_data(:,7),':b','LineWidth',2)
plot(t,x(:,2),'r','LineWidth',2)
plot(t,x(:,9),'--g','LineWidth',2)
legend('V_y (real)','V_y  (simulation)');
xlabel('Time [sec]');
ylabel('Speed [m/s]');
title('Model Test')

% figure;
subplot(2,2,3);
hold all
plot(real_data(:,1),real_data(:,8),':b','LineWidth',2)
plot(t,x(:,3),'r','LineWidth',2)
plot(t,x(:,10),'--g','LineWidth',2)
legend('\omega_c (real)','\omega_c  (simulation)');
xlabel('Time [sec]');
ylabel('Speed [rad/s]');
title('Model Test')
%--------------------------------------------------------------------------

%=Wheels' speed============================================================
% figure;
subplot(2,2,4);
hold all
plot(real_data(:,1),real_data(:,9),':b','LineWidth',2)
plot(t,x(:,4),'r','LineWidth',2)
plot(t,x(:,11),'--g','LineWidth',2)
legend('\omega_w_1 (real)','\omega_w_1  (simulation)');
xlabel('Time [sec]');
ylabel('Speed [rad/s]');
title('Model Test')
%--------------------------------------------------------------------------

figure;
hold all
plot(real_data(:,1),real_data(:,13),':b','LineWidth',2)
plot(real_data(:,1),real_data(:,6),'r','LineWidth',2)
legend('V_x (setpoint)','V_x  ');
xlabel('Time [sec]');
ylabel('Speed [m/s]');
title('Feedforward test')
title('noise test')

% figure;
% hold all
% plot(real_data2(:,1),real_data2(:,7),'r','LineWidth',2)
% plot(real_data(:,1),real_data(:,7),':b','LineWidth',2)
% legend('V_y (setpoint)','V_y  ');
% xlabel('Time [sec]');
% ylabel('Speed [m/s]');
% title('Feedforward test')
% 
% figure;
% hold all
% plot(real_data(:,1),real_data(:,8),':b','LineWidth',2)
% plot(real_data2(:,1),real_data2(:,8),'r','LineWidth',2)
% legend('\omega_c   ','\omega_c  (setpoint)');
% xlabel('Time [sec]');
% ylabel('Speed [rad/s]');
% title('Feedforward test')
