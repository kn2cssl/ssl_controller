function Xp=dynamicModel(t,X)
x=X(1:11);
global init_flag;
global A;
global B;
global C;
global K;
global G;
if init_flag ==1
    init_flag = 0;
    
    N=76/20;                     %                                           
    R=1.2;                       %ohm
    L=0.00056;                   %
    km=25.5/1000;                %Nm/A
    kn=374;                      %rpm/V
    kf=0.0001;
    kt=0.01;                      
    rw=28.5/1000;                %m           
    Jc=0.02;                     %kg/m2%     >>modeling needed
    Jm=92.5/1000/10000;          %kg/m2
    Jw=0.0000233;                %kg/m2      >>modeling needed
    rc=0.084;                    %m         
    mc=1.5;                      %kg         >>measuring needed
    a1=56.31/180*pi;             % rad 0.9827949017980069
    a2=135/180*pi;               % rad 2.356194490192345
    a3=225/180*pi;               % rad 3.926990816987241    
    a4=303.69/180*pi;            % rad 5.300390405381579  
    
    b=60/(2*pi*rw);
    Je=Jw+N^2*Jm;
    Vmax = 12.6;
    
    A=zeros(11,11);

    A(1,1) =-b*kt/mc*(sin(a1)^2+sin(a2)^2+sin(a3)^2+sin(a4)^2);
    A(1,2) =b*kt/(2*mc)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
    A(1,3) =rc*b*kt/mc*(sin(a1)+sin(a2)+sin(a3)+sin(a4));
    A(1,4) =kt*sin(a1)/mc;
    A(1,5) =kt*sin(a2)/mc;
    A(1,6) =kt*sin(a3)/mc;
    A(1,7) =kt*sin(a4)/mc;
    A(1,8) =0;
    A(1,9) =0;
    A(1,10)=0;
    A(1,11)=0;

    A(2,1)=b*kt/(2*mc)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
    A(2,2)=-b*kt/mc*(cos(a1)^2+cos(a2)^2+cos(a3)^2+cos(a4)^2);
    A(2,3)=-rc*b*kt/mc*(cos(a1)+cos(a2)+cos(a3)+cos(a4));
    A(2,4)=-kt*cos(a1)/mc;
    A(2,5)=-kt*cos(a2)/mc;
    A(2,6)=-kt*cos(a3)/mc;
    A(2,7)=-kt*cos(a4)/mc;
    A(2,8) =0;
    A(2,9) =0;
    A(2,10)=0;
    A(2,11)=0;

    A(3,1)=b*rc*kt/Jc*(sin(a1)+sin(a2)+sin(a3)+sin(a4));
    A(3,2)=-b*rc*kt/Jc*(cos(a1)+cos(a2)+cos(a3)+cos(a4));
    A(3,3)=-4*b*rc^2*kt/Jc;
    A(3,4)=-rc*kt/Jc;
    A(3,5)=-rc*kt/Jc;
    A(3,6)=-rc*kt/Jc;
    A(3,7)=-rc*kt/Jc;
    A(3,8) =0;
    A(3,9) =0;
    A(3,10)=0;
    A(3,11)=0;

    A(4,1)=rw*kt*b*sin(a1)/Je;
    A(4,2)=-rw*kt*b*cos(a1)/Je;
    A(4,3)=-rw*kt*b*rc/Je;
    A(4,4)=-rw*kt/Je-kf/Je;
    A(4,8)=N*km/Je;

    A(5,1)=rw*kt*b*sin(a2)/Je;
    A(5,2)=-rw*kt*b*cos(a2)/Je;
    A(5,3)=-rw*kt*b*rc/Je;
    A(5,5)=-rw*kt/Je-kf/Je;
    A(5,9)=N*km/Je;

    A(6,1)=rw*kt*b*sin(a3)/Je;
    A(6,2)=-rw*kt*b*cos(a3)/Je;
    A(6,3)=-rw*kt*b*rc/Je;
    A(6,6)=-rw*kt/Je-kf/Je;
    A(6,10)=N*km/Je;

    A(7,1)=rw*kt*b*sin(a4)/Je;
    A(7,2)=-rw*kt*b*cos(a4)/Je;
    A(7,3)=-rw*kt*b*rc/Je;
    A(7,7)=-rw*kt/Je-kf/Je;
    A(7,11)=N*km/Je;

    A(8,4)=-N/(L*kn);
    A(8,8)=-R/L;

    A(9,5)=-N/(L*kn);
    A(9,9)=-R/L;

    A(10,6)=-N/(L*kn);
    A(10,10)=-R/L;

    A(11,7)=-N/(L*kn);
    A(11,11)=-R/L;
    
    B=zeros(11,4);

    B(8,1) =1/L;
    B(9,2) =1/L;
    B(10,3)=1/L;
    B(11,4)=1/L;
    
    C=eye(11,11);
    
    Q=diag([1,1,0.01,0,0,0,0,0,0,0,0]);
    R=diag([1/Vmax^2,1/Vmax^2,1/Vmax^2,1/Vmax^2]);
    K=lqr(A,B,Q,R);
    
    V=diag([0.00001 0.00001 0.00001    0.1 0.1 0.1 0.1    0.01 0.01 0.01 0.01]);
    W=diag([0.001 0.001 0.001         100 100 100 100     1 1 1 1]);
    
    G=lqr(A',C',W,V)';
end


global real_data;
global r_d_size;
% u = [1;1;-1;-1];
 for e=1:r_d_size(1,1)
     if (real_data(e,1)>t*1000)
%           u=-real_data(e,2:5)'/1e+3;
          u=-[real_data(e,2)/1e+3-sign(real_data(e,2))*2.9
              real_data(e,3)/1e+3-sign(real_data(e,3))*2.9
              real_data(e,4)/1e+3-sign(real_data(e,4))*2.9
              real_data(e,5)/1e+3-sign(real_data(e,5))*2.9];
         break;
     end
 end
 
 
xp=A*x+B*u;
y=C*x;

%full observer=============================================================
y=real_data(6:16)';
xh=X(12:22);
Ah=A;
Ch=C;
Bh=B;
uh=u;
xhp=Ah*xh+Bh*uh+G*(y-Ch*xh);
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Xp=[xp ; xhp];