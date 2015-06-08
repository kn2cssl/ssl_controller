clear all
clf, echo on
tspan=[0 5];
x0=[0;0;0;0;0;0;0];
global output;
global p_overflow Setpoint_d Setpoint_last PID_Err Setpoint RPM ...
    Setpoint_change Setpoint_bridge Setpoint_miss d_last d ...
    Setpoint_track kd Mkp;
global Akp;
global setpoint;
global i;
global err;
global T;
T=zeros(0,0);
err = zeros(7,1);
setpoint = zeros(3,0);
i=zeros(4,1);
Akp=zeros(4,1);
p_overflow=zeros(4,1);
Setpoint_d=zeros(4,1);
Setpoint_last=zeros(4,1);
PID_Err=zeros(4,1);
Setpoint=zeros(4,1);
RPM  = zeros(4,1);
Setpoint_change=zeros(4,1);
Setpoint_bridge=zeros(4,1);
Setpoint_miss=zeros(4,1);
d_last=zeros(4,1);
Mkp=zeros(4,1);
d=zeros(4,1);
Setpoint_track=zeros(4,1);
kd=zeros(4,1);


[t,x] = ode45(@ssl_robot,tspan, x0, ...
    odeset('OutputFcn','odeplot','MaxStep',1e-3)); 
output

figure;
hold all
plot(t,x(:,1)*1000,'LineWidth',2)
plot(t,x(:,2)*1000,'LineWidth',2)
plot(t,x(:,3)*1000,'LineWidth',2)
plot(t,x(:,4),'LineWidth',2)
plot(t,x(:,5),'LineWidth',2)
plot(t,x(:,6),':','LineWidth',2)
plot(t,x(:,7),':','LineWidth',2)
plot(T',setpoint(1,:)'*1000,'c:','LineWidth',2)
xlabel('Time (sec)')
ylabel('State Variables')
legend('x (mm)','y (mm)','\theta (rad/1000)','\omega_1 (rpm)','\omega_2 (rpm)','\omega_3 (rpm)','\omega_4 (rpm)','setpoint (mm)')
set(findall(figure(1),'type','line'),'linewidth',2)

figure;
plot(output','DisplayName','output')
xlabel('Time (sec)')
ylabel('Voltage(v)')
set(findall(figure(3),'type','line'),'linewidth',2)

figure;
bar(log(err))
ylabel('log(abs(err))')
xlabel('space states in LQR')
