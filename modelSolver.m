clear all
clc
clf, echo on
tspan=[0 2];
x0=[0;0;0;0;0;0;0;0;0;0;0];
[t,x] = ode45(@dynamicModel,tspan, x0, ...
    odeset('OutputFcn','odeplot','MaxStep',20*1e-1)); 

figure;
hold all

%model output
plot(t,x(:,1)*1000,'r','LineWidth',2)
plot(t,x(:,2)*1000,'b','LineWidth',2)
plot(t,x(:,3)*1000,'c','LineWidth',2)
plot(t,x(:,4),'g','LineWidth',2)
plot(t,x(:,5),'g','LineWidth',2)
plot(t,x(:,6),'g','LineWidth',2)
plot(t,x(:,7),'g','LineWidth',2)
plot(t,x(:,8),'c','LineWidth',2)
plot(t,x(:,9),'c','LineWidth',2)
plot(t,x(:,10),'c','LineWidth',2)
plot(t,x(:,11),'c','LineWidth',2)

% Full order observer output
% plot(t,x(:,9)*1000,'r:','LineWidth',2)
% plot(t,x(:,10)*1000,'b:','LineWidth',2)
% plot(t,x(:,11)*1000,'c:','LineWidth',2)
% plot(t,x(:,11),'r:','LineWidth',2)
% plot(t,x(:,12),'r:','LineWidth',2)
% plot(t,x(:,13),'r:','LineWidth',2)
% plot(t,x(:,13),'r:','LineWidth',2)
% plot(t,x(:,14),'r:','LineWidth',2)

xlabel('Time (sec)')
ylabel('State Variables')
legend('setpoint (vx)','setpoint (vy)','setpoint (wr)','Vx (mm/s)','State Variables')
set(findall(figure(1),'type','line'),'linewidth',2)
