clear all
clf, echo on
tspan=[0 1];
x0=[0;0;0;0;0;0;0];
[t,x] = ode45(@ssl_robot,tspan, x0, ...
    odeset('OutputFcn','odeplot','MaxStep',1e-2)); 


figure;
hold all
plot(t,x(:,1)*1000,'LineWidth',2)
plot(t,x(:,2)*1000,'LineWidth',2)
plot(t,x(:,3)*1000,'LineWidth',2)
plot(t,x(:,4),'LineWidth',2)
plot(t,x(:,5),'LineWidth',2)
plot(t,x(:,6),':','LineWidth',2)
plot(t,x(:,7),':','LineWidth',2),grid
xlabel('Time (sec)')
ylabel('State Variables')
legend('x (mm)','y (mm)','\theta (rad/1000)','\omega_1 (rpm)','\omega_2 (rpm)','\omega_3 (rpm)','\omega_4 (rpm)')
set(findall(figure(1),'type','line'),'linewidth',2)
% 
% figure;
% subplot(2,1,1)
% plot(t,x(:,1),'r-',t,x(:,2),'g-',t,x(:,3),'b-')
% subplot(2,1,2)
% plot(t,x(:,4),'r-',t,x(:,5),'g-',t,x(:,6),'c-',t,x(:,7),'m-'),grid
% xlabel('Time (sec)')
% ylabel('State Variables')
% legend('x (m)', '\theta (rad)')