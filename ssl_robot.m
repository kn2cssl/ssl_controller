function Xp=ssl_robot(t,X)
global flag
global T;
%===================================================dynamic model of system
x=X(1:7);

xn=x+(2*rand(7,1)-ones(7,1)).*x/10;%noisy data of sensors

xm=xn(1:6);%
xl=xn;
global Gl
si=X(15);
xl(7,1)=(si+Gl*xm);

xh=X(8:14);% Full order kalman observer output

N=76/20;                     %                                           
res=1.2;                     %ohm
km=25.5/1000;                %Nm/A
kn=374;                      %rpm/V
kf=0.0001;                   %unknown 
ks=0.1;                      %unknown
r=28.5/1000;                 %m           
J=0.0192;                    %kg/m2%           >>modeling needed
Jm=92.5/1000/10000;          %kg/m2
Jw=0.0000233;                %kg/m2        >>modeling needed
d=0.084;                     %m         
M=1.5;                       %kg         >>need measuring
a1=56.31/180*pi;   % rad 0.9827949017980069
a2=135/180*pi;     % rad 2.356194490192345
a3=225/180*pi;     % rad 3.926990816987241    
a4=303.69/180*pi;  % rad 5.300390405381579     

g1=0;20.01/180*pi; % rad 0.3492403833240653   
g2=0/180*pi;     % rad 0 
g3=0/180*pi;     % rad 0  
g4=0;20.01/180*pi; % rad 0.3492403833240653   

%wheels' location
%z=[a1 a2-a1 a3-a2 a4-a3 2*pi-a4];
%pie(z);

A=zeros(7,7);
b=60/(2*pi*r);

A(1,1)=-b*ks/M*(sin(a1)^2+sin(a2)^2+sin(a3)^2+sin(a4)^2);
A(1,2)=b*ks/(2*M)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
A(1,3)=d*b*ks/M*(sin(a1)*cos(g1)+sin(a2)*cos(g2)+sin(a3)*cos(g3)+sin(a4)*cos(g4));
A(1,4)=-ks*sin(a1)/M;
A(1,5)=-ks*sin(a2)/M;
A(1,6)=-ks*sin(a3)/M;
A(1,7)=-ks*sin(a4)/M;

A(2,1)=b*ks/(2*M)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
A(2,2)=-b*ks/M*(cos(a1)^2+cos(a2)^2+cos(a3)^2+cos(a4)^2);
A(2,3)=-d*b*ks/M*(cos(a1)*cos(g1)+cos(a2)*cos(g2)+cos(a3)*cos(g3)+cos(a4)*cos(g4));
A(2,4)=ks*cos(a1)/M;
A(2,5)=ks*cos(a2)/M;
A(2,6)=ks*cos(a3)/M;
A(2,7)=ks*cos(a4)/M;

A(3,1)=b*d*ks/J*(sin(a1)*cos(g1)+sin(a2)*cos(g2)+sin(a3)*cos(g3)+sin(a4)*cos(g4));
A(3,2)=-b*d*ks/J*(cos(a1)*cos(g1)+cos(a2)*cos(g2)+cos(a3)*cos(g3)+cos(a4)*cos(g4));
A(3,3)=-b*d^2*ks/J*(cos(g1)*cos(g1)+cos(g2)*cos(g2)+cos(g3)*cos(g3)+cos(g4)*cos(g4));
A(3,4)=d*ks/J*cos(g1);
A(3,5)=d*ks/J*cos(g2);
A(3,6)=d*ks/J*cos(g3);
A(3,7)=d*ks/J*cos(g4);

Je=Jw+N^2*Jm;
ca=N*km/(Je*res);

A(4,1)=-r*ks*b*sin(a1)/Je;
A(4,2)=r*ks*b*cos(a1)/Je;
A(4,3)=r*ks*b*d/Je*cos(g1);
A(4,4)=-ca*N/kn-kf/Je-r*ks/Je;

A(5,1)=-r*ks*b*sin(a2)/Je;
A(5,2)=r*ks*b*cos(a2)/Je;
A(5,3)=r*ks*b*d/Je*cos(g2);
A(5,5)=-ca*N/kn-kf/Je-r*ks/Je;

A(6,1)=-r*ks*b*sin(a3)/Je;
A(6,2)=r*ks*b*cos(a3)/Je;
A(6,3)=r*ks*b*d/Je*cos(g3);
A(6,6)=-ca*N/kn-kf/Je-r*ks/Je;

A(7,1)=-r*ks*b*sin(a4)/Je;
A(7,2)=r*ks*b*cos(a4)/Je;
A(7,3)=r*ks*b*d/Je*cos(g4);
A(7,7)=-ca*N/kn-kf/Je-r*ks/Je;

B=zeros(7,4);

B(4,1)=ca;
B(5,2)=ca;
B(6,3)=ca;
B(7,4)=ca;

C=eye(7,7);
 
D=zeros(7,4);
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

% % %defining the state space 
% % SSLRobot = ss(A,B,C,D); 
% % %determination of transfer matrix 
% % syms s; 
% % H=C*inv(s*eye(7)-A)*B+D 
% % zpk(SSLRobot) tf(SSLRobot)

% solve(det(s*eye(7,7)-A))
% if(t<40)
%     U=[12;12;-12;-12]*sin(t/10);%[0;0;0;0];%
% end
% if(t>40)
%     U=[10;10;-10;-10];
% end
% 
% if(t>60)
%     U=[10;10;-10;-10]*0;
% end

%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::setpoints
if (t<2)
    Vd=[2;0;0];
end
if(2<t)
     Vd=[0;0;4];[2*sin(2*t);0;0];
end
if(4<t)
     Vd=[0;2;0];
end
%!!!!!!!!!!!!!!

%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%==kinematics rules that should be considered for specifying desierd output
 Yd=[Vd
     (-Vd(1,1)*sin(a1)+Vd(2,1)*cos(a1)+Vd(3,1)*cos(g1)*d)*b
     (-Vd(1,1)*sin(a2)+Vd(2,1)*cos(a2)+Vd(3,1)*cos(g2)*d)*b
     (-Vd(1,1)*sin(a3)+Vd(2,1)*cos(a3)+Vd(3,1)*cos(g3)*d)*b 
     (-Vd(1,1)*sin(a4)+Vd(2,1)*cos(a4)+Vd(3,1)*cos(g4)*d)*b
 ];

if(6<t)
     Yd=[0;0;0;1000;0;0;0];
end
if(8<t)
     Yd=[0;0;0;0;1000;0;0];
end
if(10<t)
     Yd=[0;0;0;0;0;1000;0];
end
if(12<t)
     Yd=[0;0;0;0;0;0;1000];
end


%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


%============================================================state feedback
global K
if flag == 0
% % % % pole placement
%real poles of system:[ -24.04, -3.76, -56.00, -73.02, -19.04, -4.28, -190.84]
% pd= [ -10, -11, -16, -16, -12, -15, -17]*1.69;
% Kpole=place(A,B,pd);

% % % % LQR
Q=diag([1/2,1/2,1/2,1/1000^2,1/1000^2,1/1000^2,1/1000^2]);
R=diag([1/12.6^2,1/12.6^2,1/12.6^2,1/12.6^2]);
Klqr=lqr(A,B,Q,R);
K=Klqr;
end

xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd;
ud=-inv(B'*B)*B'*A*xd;
du=-K*(x-xd);%for simulating controller with unnoisy data 
%du=-K*(xl-xd);%for simulating controller and observer:(x-xd)|(xh-xd)|(xl-xd)
u=ud+du;
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%;;;;;;
%PD_CTS
%%%%%%%

%=============================================================pi controller
% global i
% kp=1;
% ki=.000009;
% i = i+(xd(4:7)-x(4:7))*ki;
% p = (xd(4:7)-x(4:7))*kp;
% u= p+i;
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%================================================output control(Saturation)
for e=1:4
    if (abs(u(e))>12.6)
        u(e)=sign(u(e))*12.6;
    end
end
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

xp=A*x+B*u;
% xp=[A B]*[x;u];

yn=C*xn; 
y=C*x;

%RLS=======================================================================
global p;
global q;
global theta;
yk=xp';
xk=[x;u]';
%u=

%theta=[A B]'
landa=0.99999;
   p=p/landa-(p*(xk'*xk)*p)/(landa+xk*p*xk')/landa
   q=q+xk'*yk;     
                        %x1 
                        %x2
                        %x3
                        %x4
                        %x5
             %     q=q+[%x6]*[y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11]
                        %x7
                        %x8
                        %x9
                        %x10
                        %x11
                      
   theta(:,:)=p*q;



%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%full observer=============================================================
%::nose 10%
y=yn;
%::
xh=X(8:14);
Ah=A;
Ch=C;
Bh=B;
% Dh=D;
uh=u;

V=diag([0.005^2 0.005^2 0.001^2 100 100 100 100]);
W=diag([1 1 1 0.01 0.01 0.01 0.01]);

% V=diag([0.05^2 0.05^2 0.01^2 100 100 100 100]);
% W=diag([0 0 0 0 0 0 0]);

global G
if flag==0
    G=lqr(A',C',W,V)';
end


xhp=A*x+B*u;%!!!!!!Ah*xh+Bh*uh+G*(y-Ch*xh);
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%luenberger observer=======================================================
xm=xn(1:6);
si=X(15);

Amm=A(1:6,1:6);
Amu=A(1:6,7);
Aum=A(7,1:6);
Auu=A(7,7);

Bm=B(1:6,:);
Bu=B(7,:);
Dh=D;
uh=u;
if flag == 0
Vl=diag([0.05^2 0.05^2 0.01^2 100 100 100]);
Wl=diag([0.01]);

Gl=lqr(Auu',Amu',Wl,Vl)';
end

sip=0;%!!!!!!(Auu-Gl*Amu)*si+(Auu*Gl+Aum-Gl*Amm-Gl*Amu*Gl)*xm+(Bu-Gl*Bm)*uh;
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Xp=[xp;xhp;sip];
%Xp=[xp];

global setpoint;
setpoint=[setpoint;Vd'];


T=[T;t];

global output
output = [output;u'];

global xu
xu = [xu;(si+Gl*xm)'];

global xno
xno = [xno;xn'];

xl=xn;
xl(7,1)=(si+Gl*xm);

global err
err = err + abs(x-xh);

flag=1;
end