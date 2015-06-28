%setpoint
Vx = 4;% 4m/s
Vy = 0;% 4m/s
Vw = 0;% 4m/s
%defining constants
n=3.8;
R=1.2;
km=25.5/1000;
r=25.5/1000;
J=0.0192;
Jw=0.001;
Je=0.00233;
d=0.08;
m=1.5;
a1=57/180*pi;a2=135/180*pi;a3=225/180*pi;a4=303/180*pi;
Bx=0;
By=1;
Bw=1;
g1=n*km*Jw/(R*r*Je);
g2=n*km/(R*r);
h1=g1*km*n/m;
h2=d*g1*km*n/J;
h3=km*n*g2;
h4=d*g1/J;
%defining state matrixes
A=[-Bx 0 0 h1*sin(a1) h1*sin(a2) h1*sin(a3) h1*sin(a4)
    0 -By 0 -h1*cos(a1) -h1*cos(a2) -h1*cos(a3) -h1*cos(a4)
    0 0 -Bw -h2 -h2 -h2 -h2
    0 0 0 -h3 0 0 0
    0 0 0 0 -h3 0 0
    0 0 0 0 0 -h3 0
    0 0 0 0 0 0 -h3]
B=[-sin(a1)*g1/m -sin(a2)*g1/m -sin(a3)*g1/m -sin(a4)*g1/m
   cos(a1)*g1/m cos(a2)*g1/m cos(a3)*g1/m cos(a4)*g1/m
   h4 h4 h4 h4
   g2 0 0 0
   0 g2 0 0
   0 0 g2 0
   0 0 0 g2]
C=eye(7)
D=0

%investigating controllability and observability
O=[C;C*A;C*A^2;C*A^3;C*A^4;C*A^5;C*A^6];    % =obsv(A,C)
rank(O)

Co=[B A*B A^2*B A^3*B A^4*B A^5*B A^6*B];   % =ctrb(A,B)
rank(Co)

%similarity transformations (transforming to jordan's form)
[T,Aj]=jordan(A);
Bj=inv(T)*B;
Cj=C*T;
% Q1=inv(B'*B)*B'
% Q2=-Q1*A
%defining the state space
SSLRobot = ss(A,B,C,D);
%determination of transfer matrix
syms s;
H=C*inv(s*eye(7)-A)*B+D;
zpk(SSLRobot);
tf(SSLRobot);


t=0:0.01:0.01; N=max(size(t));

u = ones(2,1);%;gensig('square',10,100,0.01); %u=-6*u+3;
u=u';
U=[-u;-u;u;u]*0;
x0= [0,0,0,0,0,0,0];
err=100;

i=1;

x=zeros(10000,7);
k=100;
Vx=100
while(abs(err)>5)
    i=1+i
    [y,t,x(i:i+1,:)]=lsim(SSLRobot,U,t,x(i,:));
    err=Vx-x(i);
    x(i)
    U=[k*err k*err;k*err k*err;k*err k*err;k*err k*err;]
    
end




plot(x(:,1),'C',x(:,2),'R-.',x(:,3),'G:',x(:,4),'B--',x(:,5),'k-.',x(:,6),'y-',x(:,7),'R.-'), grid
set(findall(figure(1),'type','line'),'linewidth',2)
xlabel('Time (sec)')
ylabel('State variables')
legend('Vx', 'Vy','\omega','\omega1','\omega2','\omega3','\omega4')

% phi=ilaplace(inv(s*eye(7)-A))
% syms time
% phi2(time)=expm(A*time)
% Y=phi2(10)*x0




   