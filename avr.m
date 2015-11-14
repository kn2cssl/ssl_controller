N=76/20;                     %
res=1.2;                     %ohm
km=25.5/1000;                %Nm/A
kn=374;                      %rpm/V
kf=0.001;                                  %unknown
ks=0.01;                                    %unknown
r=28.5/1000;                 %m
J=0.0192;                    %kg/m2%           >>modeling needed
Jm=92.5/1000/10000;          %kg/m2
Jw=0.0000233;                %kg/m2        >>modeling needed
d=0.084;                     %m
M=1.8;                       %kg         >>need measuring
a1=56.31/180*pi;   % rad 0.9827949017980069
a2=135/180*pi;     % rad 2.356194490192345
a3=225/180*pi;     % rad 3.926990816987241
a4=303.69/180*pi;  % rad 5.300390405381579
g1=20.01/180*pi; % rad 0.3492403833240653
g2=0/180*pi;     % rad 0
g3=0/180*pi;     % rad 0
g4=20.01/180*pi; % rad 0.3492403833240653
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
A(3,3)=-4*b*d^2*ks/J*(cos(g1)*cos(g1)+cos(g2)*cos(g2)+cos(g3)*cos(g3)+cos(g4)*cos(g4));
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
Vd=[2;0;0];
cos_alpha = cos(0);
sin_alpha = sin(0);
Vx = (  Vd(1,1) * cos_alpha + Vd(2,1) * sin_alpha ) ;
Vy = ( -Vd(1,1) * sin_alpha + Vd(2,1) * cos_alpha ) ;
Wr =   Vd(3,1) ;
Yd=[Vx
Vy
Wr
(-Vx*sin(a1)+Vy*cos(a1)+Wr*cos(g1)*d)*b
(-Vx*sin(a2)+Vy*cos(a2)+Wr*cos(g2)*d)*b
(-Vx*sin(a3)+Vy*cos(a3)+Wr*cos(g3)*d)*b
(-Vx*sin(a4)+Vy*cos(a4)+Wr*cos(g4)*d)*b
];
xx=[0 0 0 0 0 0 0];
z=[0;0;0;0;0;0;0];

V=diag([0.005^2 0.005^2 0.001^2 100 100 100 100]);
W=diag([1 1 1 0.01 0.01 0.01 0.01]);
G=lqr(A',C',W,V)';

Q=diag([1/2,1/2,1/2,1/1000^2,1/1000^2,1/1000^2,1/1000^2]);
R=diag([1/12.6^2,1/12.6^2,1/12.6^2,1/12.6^2]);
Klqr=lqr(A,B,Q,R);
K=Klqr;

for m=0:size(t,1)-2
xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd;
ud=-inv(B'*B)*B'*A*xd;
du=-K*(z-xd);%for simulating controller with unnoisy data
%du=-K*(xl-xd);%for simulating controller and observer:(x-xd)|(xh-xd)|(xl-xd)
u=ud+du;
for e=1:4
if (abs(u(e))>12.6)
u(e)=sign(u(e))*12.6;
end
end


%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
z=(A*z+B*u+G*(yo(:,m+1)-C*z))*0.00378+z;
xx=[xx;z'];
end
