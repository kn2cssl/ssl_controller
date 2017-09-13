clear
close
clc
real_data = importdata('ssl-d-13960619-1-7ss-trapezoidal-1.txt');
r_d_size =  size(real_data);
real_data = [real_data(:,1:8)/1000, real_data(:,9:12), real_data(:,13:16)/1000];

real_data2 = importdata('ssl-d-13960611-1-7ss-lowpass-2.txt');
r_d_size2 =  size(real_data2);
real_data2 = [real_data2(:,1:8)/1000, real_data2(:,9:12), real_data2(:,13:16)/1000];

fil=0;
for n=2:r_d_size(1,1)
fil=[fil;fil(n-1)+(real_data(n,2)-fil(n-1))*0.3];
end
hold all
% plot(real_data(:,1),real_data(:,8),'r','LineWidth',2)
% legend('V_x (real)')
plot(real_data(:,1),real_data(:,2),'b','LineWidth',2)
% plot(real_data(:,1),fil,':r','LineWidth',2)
plot(real_data(:,1),real_data(:,6),':g','LineWidth',2)
plot(real_data2(:,1),real_data2(:,6),'--r','LineWidth',2)

legend('V_x (real)','V_x  (low pass filter)','V_x  (kalman filter)');
xlabel('Time [sec]');
ylabel('Speed [m/s]');

% legend('\omega_c (real)','\omega_c  (low pass filter)','\omega_c  (kalman filter)');
% xlabel('Time [sec]');
% ylabel('Speed [rad/s]');

a=0;
v=0;
x=0;
V=0;
X=0;
A=0;
for n=0:750
    if(n<250)
        a=1;
    elseif(n<500)
        a=0;
    else
        a=-1;
    end
    v=v+a;
    x=x+v;
    V=[V;v];
    X=[X;x];
    A=[A;a];
end

hold all
plot(A*100,':r','LineWidth',2)
plot(V/2,'g','LineWidth',2)
plot(X/1000,'--b','LineWidth',2)
legend('Accel and Deccel','Velocity','Distance');
