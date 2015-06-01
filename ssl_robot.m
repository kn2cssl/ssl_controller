function xp=ssl_robot(t,X,U)
x=X(1:7);
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
a1=56.31/180*pi;   % rad
a2=135/180*pi;     % rad
a3=225/180*pi;     % rad     
a4=303.69/180*pi;  % rad     

g1=20.01/180*pi; % rad   
g2=0/180*pi;     % rad   
g3=0/180*pi;     % rad   
g4=20.01/180*pi; % rad   

%z=[a1 a2-a1 a3-a2 a4-a3 2*pi-a4];
%pie(z);
% sim('hgjh',[0 10]);
% out1.time
% out1.signals.value(
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
%%kinematics rules that should be considered
%%for Specifying desierd output
Vd=[8^.5;8^.5;0];
 Yd=[Vd;(-Vd(1,1)*sin(a1)+Vd(2,1)*cos(a1)+Vd(3,1)*sin(g1)*d)*b
     (-Vd(1,1)*sin(a2)+Vd(2,1)*cos(a2)+Vd(3,1)*sin(g2)*d)*b
     (-Vd(1,1)*sin(a3)+Vd(2,1)*cos(a3)+Vd(3,1)*sin(g3)*d)*b 
     (-Vd(1,1)*sin(a4)+Vd(2,1)*cos(a4)+Vd(3,1)*sin(g4)*d)*b
 ]
 
% % x.=A*dx+B*du
% % dy=C*dx
% % 
% % 0=A*xd+B*ud
% % yd=C*xd

% % xd=inv(C'*C)*C'*yd
% % ud=-inv(B'*B)*B'*A*xd



% % Q=diag([1,1,1,1,1,1,1]);
% % R=diag([.01,.01,.01,.01]);
Q=diag([1,1,1,1,1,1,1]);
R=diag([1/144,1/144,1/144,1/144]);
pd= [ -25, -5, -60, -80, -20, -10, -215];%[ -25, -25, -60, -80, -25, -25, -100];
 
% Klqr=lqr(A,B,Q,R);
% U=(inv(B'*B)*B'*A*inv(C'*C)*C'+Klqr*inv(C'*C)*C')*Yd;
%  
% Acl=A-B*Klqr;
% 
% xp=Acl*X+B*U;
Klqr1=place(A,B,pd)*0;% pole correction
Klqr=lqr(A,B,Q,R);%lqr


xd=inv(C'*C)*C'*Yd;
ud=-inv(B'*B)*B'*A*xd;
du=-Klqr1*(x-xd);
u=ud+du

xp=A*x+B*u;
 
y=C*x;
end