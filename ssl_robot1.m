function xp=ssl_robot(t,X,U)

N=3.8; %                                                                   >>need measuring
R=1.2;%ohm
km=25.5/1000; %Nm/A
kn=374; %rpm/V
kf=0.0001;%unknown                                                         >>modeling needed
ks=0.1;%unknown
r=25.5/1000; %m                                                            >>need measuring
J=0.0192; %kg/m2%           >>modeling needed
Jm=92.5/1000/10000; %kg/m2
Jw=0.0000233; %kg/m2        >>modeling needed
d=0.08393; %m         
M=1.5; %kg         >>need measuring
a1=56.31/180*pi;     % rad
a2=135/180*pi;     % rad
a3=225/180*pi;     % rad     
a4=303.69/180*pi;     % rad     

g1=20.01/180*pi;     % rad   
g2=0/180*pi;     % rad   
g3=0/180*pi;     % rad   
g4=20.01/180*pi;     % rad   

%z=[a1 a2-a1 a3-a2 a4-a3 2*pi-a4];
%pie(z);

A=zeros(7,7);
b=60/(2*pi*r);

A(1,1)=-b*ks/M*(sin(a1)^2+sin(a2)^2+sin(a3)^2+sin(a4)^2);
A(1,2)=b*ks/(2*M)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
A(1,3)=b*ks/M*(sin(a1)+sin(a2)+sin(a3)+sin(a4));
A(1,4)=-ks*sin(a1)/M;
A(1,5)=-ks*sin(a2)/M;
A(1,6)=-ks*sin(a3)/M;
A(1,7)=-ks*sin(a4)/M;

A(2,1)=b*ks/(2*M)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
A(2,2)=-b*ks/M*(cos(a1)^2+cos(a2)^2+cos(a3)^2+cos(a4)^2);
A(2,3)=-b*ks/M*(cos(a1)+cos(a2)+cos(a3)+cos(a4));
A(2,4)=ks*cos(a1)/M;
A(2,5)=ks*cos(a2)/M;
A(2,6)=ks*cos(a3)/M;
A(2,7)=ks*cos(a4)/M;

A(3,1)=b*d*ks/J*(sin(a1)+sin(a2)+sin(a3)+sin(a4));
A(3,2)=-b*d*ks/J*(cos(a1)+cos(a2)+cos(a3)+cos(a4));
A(3,3)=-4*b*d*ks/J;
A(3,4)=d*ks/J;
A(3,5)=d*ks/J;
A(3,6)=d*ks/J;
A(3,7)=d*ks/J;

Je=Jw+N^2*Jm;
ca=N*km/(Je*R);

A(4,1)=-r*ks*b*sin(a1)/Je;
A(4,2)=r*ks*b*cos(a1)/Je;
A(4,3)=r*ks*b/Je;
A(4,4)=-ca*N/kn-kf/Je-r*ks/Je;

A(5,1)=-r*ks*b*sin(a2)/Je;
A(5,2)=r*ks*b*cos(a2)/Je;
A(5,3)=r*ks*b/Je;
A(5,5)=-ca*N/kn-kf/Je-r*ks/Je;

A(6,1)=-r*ks*b*sin(a3)/Je;
A(6,2)=r*ks*b*cos(a3)/Je;
A(6,3)=r*ks*b/Je;
A(6,6)=-ca*N/kn-kf/Je-r*ks/Je;

A(7,1)=-r*ks*b*sin(a4)/Je;
A(7,2)=r*ks*b*cos(a4)/Je;
A(7,3)=r*ks*b/Je;
A(7,7)=-ca*N/kn-kf/Je-r*ks/Je;

B=zeros(7,4);

B(4,1)=ca;
B(5,2)=ca;
B(6,3)=ca;
B(7,4)=ca;
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
 U=[10;10;-10;-10];


xp=A*X+B*U;
end