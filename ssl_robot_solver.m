    clear all
clf, echo on
tspan=[0 8];
% x0=[0.5;1;0.1;474;-3553;-1184;3262;0;0;0;0;0;0;0;0];
x0=[0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
% x0=[0;0;0;0;0;0;1000];
global output;
global flag
global G
global K
global Gl
global xno;
global p_overflow Setpoint_d Setpoint_last PID_Err Setpoint RPM ...
    Setpoint_change Setpoint_bridge Setpoint_miss d_last d ...
    Setpoint_track kd Mkp;
global Akp;
global setpoint;
global i;
global err;
global T;
global xu;
global real_setpiont;
global u;
global yn;
global yo
%RLS==========
global p;
global q;
global theta;
theta = zeros(11,7);
q=zeros(11,7);
p=eye(11)*1000 ;

%;;;;;;;;;;;;;
flag = 0 ;
T=zeros(0,0);
err = zeros(7,1);
Gl = zeros(1,6);
xno = zeros(0,7);
setpoint = zeros(0,3);
i=zeros(4,1);
Akp=zeros(4,1);
p_overflow=zeros(4,1);
Setpoint_d=zeros(4,1);
Setpoint_last=zeros(4,1);
PID_Err=zeros(4,1);
Setpoint=zeros(0,4);
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
    odeset('OutputFcn','odeplot','MaxStep',20*1e-1)); 

figure;
hold all

% plot(T,setpoint(:,1)'*1000,'r:','LineWidth',2)
% plot(T,setpoint(:,2)'*1000,'b:','LineWidth',2)
% plot(T,setpoint(:,3)'*1000,'c:','LineWidth',2)
%model output
plot(t,x(:,1)*1000,'r','LineWidth',2)
plot(t,x(:,2)*1000,'b','LineWidth',2)
plot(t,x(:,3)*1000,'c','LineWidth',2)
plot(t,x(:,4),'g','LineWidth',2)
plot(t,x(:,5),'g','LineWidth',2)
plot(t,x(:,6),'g','LineWidth',2)
plot(t,x(:,7),'g','LineWidth',2)
% % % % %Luenberger observer output
% plot(T,xu,'b:','LineWidth',2)
figure;
hold all
% Full order observer output
plot(t,x(:,9)*1000,'r:','LineWidth',2)
plot(t,x(:,10)*1000,'b:','LineWidth',2)
plot(t,x(:,11)*1000,'c:','LineWidth',2)
plot(t,x(:,11),'r:','LineWidth',2)
plot(t,x(:,12),'r:','LineWidth',2)
plot(t,x(:,13),'r:','LineWidth',2)
plot(t,x(:,13),'r:','LineWidth',2)
plot(t,x(:,14),'r:','LineWidth',2)



% noisy data
% plot(T,xno(:,1)*1000,'y:','LineWidth',2)
% plot(T,xno(:,2)*1000,'y:','LineWidth',2)
% plot(T,xno(:,3)*1000,'y:','LineWidth',2)
% plot(T,xno(:,4),'y:','LineWidth',2)
% plot(T,xno(:,5),'y:','LineWidth',2)
% plot(T,xno(:,6),'y:','LineWidth',2)
% plot(T,xno(:,7),'y:','LineWidth',2)


xlabel('Time (sec)')
ylabel('State Variables')
legend('setpoint (vx)','setpoint (vy)','setpoint (wr)','Vx (mm/s)','State Variables')
% legend('x (mm)','y (mm)','\theta (rad/1000)','\omega_1 (rpm)','\omega_2 (rpm)','\omega_3 (rpm)','\omega_4 (rpm)','setpoint (mm)')
% legend('Real data','Luenberger observer output','noisy data if there was a sensor')
set(findall(figure(1),'type','line'),'linewidth',2)

% controller output(voltage)
% figure;
% plot(output,'DisplayName','output')
% xlabel('Time (sec)')
% ylabel('Voltage(v)')
% set(findall(figure(3),'type','line'),'linewidth',2)

% err 
% figure;
% bar(log(err))
% ylabel('log(abs(err))')
% xlabel('system outputs : Luenberger observer output + noisy data')%Full order observer output
