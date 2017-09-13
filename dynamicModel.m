function Xp=dynamicModel(t,X)
x=X(1:7);
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
    km=25.5/1000;                %Nm/A
    kn=374;                      %rpm/V
    kf=0.0001;                   %           >>measuring needed
    kt=0.008;                    %           >>measuring needed                   
    rw=28.5/1000;                %m           
    Jc=0.00139652802;            %kg*m2%     >>SOLIDWORKS
    Jm=92.5/1000/10000;          %kg*m2
    Jw=0.00001;                  %kg*m2      >>SOLIDWORKS
    rc=0.084;                    %m         
    mc=1.5;                      %kg         >>measuring needed
    a1=56.31/180*pi;             % rad 0.9827949017980069
    a2=135/180*pi;               % rad 2.356194490192345
    a3=225/180*pi;               % rad 3.926990816987241    
    a4=303.69/180*pi;            % rad 5.300390405381579  
    
    b=60/(2*pi*rw);
    Je=Jw+N^2*Jm;
    Ca=N*km/(Je*R);
    Vmax = 12.6;
    
    A=zeros(7,7);

    A(1,1) =-b*kt/mc*(sin(a1)^2+sin(a2)^2+sin(a3)^2+sin(a4)^2);
    A(1,2) =b*kt/(2*mc)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
    A(1,3) =rc*b*kt/mc*(sin(a1)+sin(a2)+sin(a3)+sin(a4));
    A(1,4) =kt*sin(a1)/mc;
    A(1,5) =kt*sin(a2)/mc;
    A(1,6) =kt*sin(a3)/mc;
    A(1,7) =kt*sin(a4)/mc;

    A(2,1)=b*kt/(2*mc)*(sin(2*a1)+sin(2*a2)+sin(2*a3)+sin(2*a4));
    A(2,2)=-b*kt/mc*(cos(a1)^2+cos(a2)^2+cos(a3)^2+cos(a4)^2);
    A(2,3)=-rc*b*kt/mc*(cos(a1)+cos(a2)+cos(a3)+cos(a4));
    A(2,4)=-kt*cos(a1)/mc;
    A(2,5)=-kt*cos(a2)/mc;
    A(2,6)=-kt*cos(a3)/mc;
    A(2,7)=-kt*cos(a4)/mc;

    A(3,1)=b*rc*kt/Jc*(sin(a1)+sin(a2)+sin(a3)+sin(a4));
    A(3,2)=-b*rc*kt/Jc*(cos(a1)+cos(a2)+cos(a3)+cos(a4));
    A(3,3)=-4*b*rc^2*kt/Jc;
    A(3,4)=-rc*kt/Jc;
    A(3,5)=-rc*kt/Jc;
    A(3,6)=-rc*kt/Jc;
    A(3,7)=-rc*kt/Jc;

    A(4,1)=rw*kt*b*sin(a1)/Je;
    A(4,2)=-rw*kt*b*cos(a1)/Je;
    A(4,3)=-rw*kt*b*rc/Je;
    A(4,4)=-rw*kt/Je-kf/Je-Ca*N/kn;

    A(5,1)=rw*kt*b*sin(a2)/Je;
    A(5,2)=-rw*kt*b*cos(a2)/Je;
    A(5,3)=-rw*kt*b*rc/Je;
    A(5,5)=-rw*kt/Je-kf/Je-Ca*N/kn;

    A(6,1)=rw*kt*b*sin(a3)/Je;
    A(6,2)=-rw*kt*b*cos(a3)/Je;
    A(6,3)=-rw*kt*b*rc/Je;
    A(6,6)=-rw*kt/Je-kf/Je-Ca*N/kn;

    A(7,1)=rw*kt*b*sin(a4)/Je;
    A(7,2)=-rw*kt*b*cos(a4)/Je;
    A(7,3)=-rw*kt*b*rc/Je;
    A(7,7)=-rw*kt/Je-kf/Je-Ca*N/kn;
    
    B=zeros(7,4);

    B(4,1) =Ca;
    B(5,2) =Ca;
    B(6,3) =Ca;
    B(7,4) =Ca;
    
    C=eye(7,7);
    
    
    Q=diag([1/4^2,1/4^2,1/440^2,1/1300^2,1/1300^2,1/1300^2,1/1300^2]);
    R=diag([1/Vmax^2,1/Vmax^2,1/Vmax^2,1/Vmax^2]);
    K=lqr(A,B,Q,R);
    
    V=diag([0.0012 0.0012 0.03   10000 10000 10000 10000]);
    W=diag([  0.0225    0.0225    0.04   2500 2500 2500 2500]);
    
%     V=diag([0.00001 0.00001 0.00001 0.1 0.1 0.1 0.1]);
%     W=diag([0.001 0.001 0.001 100 100 100 100]);

    
    G=lqr(A',C',W,V)';
end


global real_data;
global r_d_size;
% u = [1;1;-1;-1];
 for e=1:r_d_size(1,1)
     if (real_data(e,1)>t)
%           u=-real_data(e,2:5)'/1e+3;
          u=[real_data(e,2)-sign(real_data(e,2))*2.9
             real_data(e,3)-sign(real_data(e,3))*2.9
             real_data(e,4)-sign(real_data(e,4))*2.9
             real_data(e,5)-sign(real_data(e,5))*2.9];
          y=real_data(e,6:12)';
         break;
     end
 end
 
xp=A*x+B*u;
% y=C*x;

%full observer=============================================================
xh=X(8:14);
Ah=A;
Ch=C;
Bh=B;
uh=u;
xhp=Ah*xh+Bh*uh+G*(y-Ch*xh);
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%RLS=======================================================================
global p;
global q;
global theta;
global kk;
yk=y';
xk=[y;u]';
%theta=[A B]'
landa=0.9;
for n=1:7
    q(:,n)=p(:,:,n)*xk'/(xk*p(:,:,n)*xk'+landa);
    p(:,:,n)=(eye(11)-q(:,n)*xk)*p(:,:,n)/landa;
   theta(:,n)=theta(:,n) + q(:,n) * (yk(n) - xk *  theta(:,n));
end

   kk = [kk;theta(1,1),t];
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Xp=[xp ; xhp];