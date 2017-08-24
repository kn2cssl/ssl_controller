clear all
close all
clc
clf, echo on
tspan=[0 18];
x0=[0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
global init_flag ;
init_flag = 1;
global real_data ;
global r_d_size ;
real_data = importdata('ssl-d-13950601-3.txt');
r_d_size =  size(real_data);
real_data = real_data - [ones(r_d_size(1,1),1)*real_data(1,1) zeros(r_d_size(1,1),r_d_size(1,2)-1)];
real_data = [real_data(:,1:8), -real_data(:,9:16)];
[t,x] = ode45(@dynamicModel,tspan, x0, odeset('OutputFcn','odeplot','MaxStep',20*1e-1)); 

%=Robot's speed============================================================
figure;
subplot(1,3,1);
hold all
plot(t,x(:,1),'r','LineWidth',2)
plot(real_data(:,1)/1000,real_data(:,6)/1000,':b*','LineWidth',2)
legend('V_x  (simulation)','V_x (real)');

subplot(1,3,2);
hold all
plot(t,x(:,2),'r','LineWidth',2)
plot(real_data(:,1)/1000,real_data(:,7)/1000,':b*','LineWidth',2)
legend('V_y  (simulation)','V_y (real)');

subplot(1,3,3);
hold all
plot(t,x(:,3),'r','LineWidth',2)
plot(real_data(:,1)/1000,real_data(:,8)/1000,':b*','LineWidth',2)
legend('\omega_c  (simulation)','\omega_c (real)');
%--------------------------------------------------------------------------

%=Wheels' speed============================================================
figure;
subplot(2,2,1);
hold all
plot(t,x(:,4),'r','LineWidth',2)
plot(real_data(:,1)/1000,real_data(:,9),':g*','LineWidth',2)
legend('\omega_w_1  (simulation)','\omega_w_1 (real)');
%--------------------------------------------------------------------------

%=Currents=================================================================
figure;
subplot(2,2,1);
hold all
plot(t,x(:,8),'r','LineWidth',2)
plot(real_data(:,1)/1000,real_data(:,13)/1000,':c*','LineWidth',2)
legend('i_m_1  (simulation)','i_m_1 (real)');
%--------------------------------------------------------------------------

% plot(t,x(:,5),'g','LineWidth',2)
% plot(t,x(:,6),'g','LineWidth',2)
% plot(t,x(:,7),'g','LineWidth',2)

% % plot(t,x(:,9),'o','LineWidth',2)
% % plot(t,x(:,10),'o','LineWidth',2)
% % plot(t,x(:,11),'o','LineWidth',2)

% plot(real_data(:,1)/1000,real_data(:,10),':g*','LineWidth',2)
% plot(real_data(:,1)/1000,real_data(:,11),':g*','LineWidth',2)
% plot(real_data(:,1)/1000,real_data(:,12),':g*','LineWidth',2)

% Full order observer output
% plot(t,x(:,12)*1000,'--ro','LineWidth',2)
% plot(t,x(:,11)*1000,'--bo','LineWidth',2)
% plot(t,x(:,13)*1000,'--co','LineWidth',2)
% plot(t,x(:,11),'r:','LineWidth',2)
% plot(t,x(:,12),'r:','LineWidth',2)
% plot(t,x(:,13),'r:','LineWidth',2)
% plot(t,x(:,13),'r:','LineWidth',2)
% plot(t,x(:,14),'r:','LineWidth',2)

% xlabel('Time (sec)')
% ylabel('State Variables')