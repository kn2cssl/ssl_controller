function xp=ssl_robot(t,X)
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
%%kinematics rules that should be considered
%%for Specifying desierd output
if (t<2)
    Vd=[2;0;0];
end
if(2<t)
     Vd=[2*sin(2*t);0;0];
end
global setpoint;
setpoint=[setpoint Vd];

global T;
T=[T t];

 Yd=[Vd
     (-Vd(1,1)*sin(a1)+Vd(2,1)*cos(a1)+Vd(3,1)*sin(g1)*d)*b
     (-Vd(1,1)*sin(a2)+Vd(2,1)*cos(a2)+Vd(3,1)*sin(g2)*d)*b
     (-Vd(1,1)*sin(a3)+Vd(2,1)*cos(a3)+Vd(3,1)*sin(g3)*d)*b 
     (-Vd(1,1)*sin(a4)+Vd(2,1)*cos(a4)+Vd(3,1)*sin(g4)*d)*b
 ];
 
% % x.=A*dx+B*du
% % dy=C*dx
% % 
% % 0=A*xd+B*ud
% % yd=C*xd
% % xd=inv(C'*C)*C'*yd
% % ud=-inv(B'*B)*B'*A*xd


% % % % pole placement
%[ -52854702807779/2199023255552, -3.7553462935397881987443953581941, -55.995550907591353432617218795069, -73.021818929171520419536370284645, -19.043391935471979765709333978373, -4.2801974523012689996128238609308, -190.84314951365898360283971358624]
%[ -24.04, -3.76, -56.00, -73.02, -19.04, -4.28, -190.84]
%[ -6, -3, -16, -17, -13, -3, -19];test it
pd= [ -10, -11, -16, -16, -12, -15, -17]*1.69;%[ -20, -1, -50, -70, -15, -2, -180];%real poles of system:[ -24, -4, -56, -73, -19, -4, -191]
Kpole=place(A,B,pd);
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

% % % % LQR
Q=diag([1/2,1/2,1/2,1/1000^2,1/1000^2,1/1000^2,1/1000^2]);
R=diag([1/12.6^2,1/12.6^2,1/12.6^2,1/12.6^2]);
Klqr=lqr(A,B,Q,R);
%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


xd=inv(C'*C)*C'*Yd;
ud=-inv(B'*B)*B'*A*xd;
du=-Kpole*(x-xd);
u=ud+du;

%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
global p_overflow Setpoint_d Setpoint_last PID_Err Setpoint RPM Setpoint_change Setpoint_bridge Setpoint_miss d_last d Setpoint_track kd Mkp;
    kp=.20; %%base Mkp(ii) for setpoints over 500 rpm
	kp2=1;  %%base Mkp(ii) for setpoints below 500 rpm
	pwm_top = 255;
	lim1 = 15; %%this limit determine when Mkp(ii) should increase ,also when kd should change.
	lim2 = 10; %%this limit determine accuracy of rpm 
	lim3 = 300 ; %% setpont_bridge limit : err larger than lim3 
	lim4 = 400;
    for ii=1:4
	Setpoint(ii) = Yd(ii+3) ;
    d(ii)=X(ii+3)-RPM(ii);
    RPM(ii) = X(ii+3);
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%stage.1 : input stage
	% :)
	Setpoint_d(ii) = Setpoint(ii) - Setpoint_last(ii) ;
	PID_Err(ii) = Setpoint(ii) - RPM(ii) + 15 *sign(Setpoint);
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%stage.2 : status determination  
	% in this stage few conditions are specified which will be used in next stage.
		if (Setpoint_change(ii) == 1) && ( abs(PID_Err(ii)) > lim3)
		
			Setpoint_bridge(ii) = 1;
        end
		
		if (Setpoint_bridge(ii) == 1 && abs(PID_Err(ii)) < lim3)
		
			Setpoint_bridge(ii) = 0;
			Setpoint_miss(ii) = 1;
        end
		
		if (Setpoint_miss(ii) == 1 && sign(d_last(ii)) ~= sign(d(ii)))
		
			Setpoint_miss(ii) = 0;
			Setpoint_track(ii) = 1;
        end

		
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%stage.3 : Mkp(ii) & kd tuning

     if (p_overflow(ii) == 0)
            
		
         if (abs(d(ii)) < 15 && abs(PID_Err(ii)) > lim4 && abs(RPM(ii))>10)
            Mkp(ii)=Mkp(ii)+.003;
         end
            
         if (abs(d(ii)) < 15 && abs(PID_Err(ii)) < lim4 && abs(PID_Err(ii)) > lim1 &&  abs(RPM(ii))>10 )
            Mkp(ii)=Mkp(ii)+.001;
         end
     end

	if (abs(RPM(ii)) > abs(Setpoint(ii)))
		if (abs(PID_Err(ii)) < lim4 && abs(PID_Err(ii)) > lim2 && abs(RPM(ii))>10  && abs(Setpoint(ii)) > 499 )
          Mkp(ii)=Mkp(ii)-.007;  
        end
        
        if (abs(PID_Err(ii)) < lim4 && abs(PID_Err(ii)) > lim2 && abs(Setpoint(ii)) < 499 ) 
            Mkp(ii)=Mkp(ii)-.0001;
        end
        
        if (Mkp(ii) < kp )
            Mkp(ii) = kp ;
        end
    end
		
    if (abs(Mkp(ii) * PID_Err(ii)) > pwm_top)

        Mkp(ii) = abs((pwm_top) / PID_Err(ii)) ;
    end
		
    
%     if (abs(RPM(ii))<50)
% 	
%         if (abs(Setpoint(ii)) > 499)
% 		
% 		Mkp(ii) = kp;
%         
%         else
% % 		Mkp(ii) = kp2;
% %         end
%         
% 		
%     end
	
% 	if (abs(Setpoint(ii))<500)
% 	
% 		Mkp(ii) = 1;
%     end
	
	if (abs(Setpoint_d(ii)) > abs(d(ii)) && abs(PID_Err(ii)) > 200)
	
		Mkp(ii) = Mkp(ii) + abs(Setpoint_d(ii) - d(ii))*.01;
    end
	

	
	if (Setpoint_miss(ii))
	
		kd(ii) = 50 ;
    end
	
	if (Setpoint_track(ii))
	
		kd(ii) = 2 ;
		if (abs(Setpoint(ii)) < 500)
		
			kd(ii) = 1 ;
		
        end
    end
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%stage.4 : PD controller
	%here we have a conventional pd controller  
	p = PID_Err(ii) * Mkp(ii)
	
	p_overflow(ii) = 0;
	if (abs(p) > pwm_top)
	
		p = sign(p) * pwm_top ;
		p_overflow(ii) = 1;
    end
	
% 	d=(d>2400)?(2400):d;
% 	d=(d<-2400)?(2400):d;
	
	PID =p -(d(ii) * kd(ii)) ;
	
	if(PID>pwm_top)
	PID=pwm_top;
    end
	if( PID<-pwm_top)
	PID=-pwm_top;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%stage.5 : data storage
	% :)
    %PID_last = PID ;
	%p_last = p;
	%PID_Err_last = PID_Err ;
	Setpoint_change(ii) = 0;
	if (Setpoint_last(ii) ~= Setpoint(ii) )
	
		Setpoint_change(ii) = 1;
		Setpoint_track(ii) = 0;
    end
	Setpoint_last(ii) = Setpoint(ii) ;
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%stage.6 : output stage
	% the controller returns pw "if" term prevents robot from vibration when it should halt.
% 	if((Setpoint)==0 && abs(RPM-(Setpoint))<10)
% 	return 0;
% 		
% 	return PID ;
    u(ii)= PID*12.6/255;
    end
    global Akp
	Akp=[Akp Mkp];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % % % pi controller
% global i
% kp=1;
% ki=.000009;
% i = i+(xd(4:7)-x(4:7))*ki;
% p = (xd(4:7)-x(4:7))*kp;
% u= p+i;

% % % % Saturation
for e=1:4
    if (abs(u(e))>12.6)
        u(e)=sign(u(e))*12.6;
    end
end

xp=A*x+B*u;
 
y=C*x;

global output
output = [output u];

global err
err = err + abs(x-xd);

end