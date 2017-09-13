clear
clc
real_data = importdata('ssl-d-13960608-5-forRLS.txt');
r_d_size =  size(real_data);
real_data = [real_data(:,1:8)/1000, real_data(:,9:12), real_data(:,13:16)/1000];

global p;
global q;
global theta;
global kk;
%RLS=======================================================================
kk = [0,0];
theta = zeros(11,7);
q=zeros(11,7);
for n = 1:11
    p(:,:,n)=eye(11)*1000;
end
yk =0;
xk=0;
ykk =[0,0,0,0,0,0,0];
for m=51:r_d_size
    ykNew=real_data(m,6:12)';
xkNew=[real_data(m-50,6:12) real_data(m-10,2:5)];
yk = (ykNew-yk)*0.1+yk;
xk = (xkNew-xk)*0.1+xk;
%theta=[A B]'
landa=.9;
    for n=1:7
        q(:,n)=p(:,:,n)*xk'/(xk*p(:,:,n)*xk'+landa);
        p(:,:,n)=(eye(11)-q(:,n)*xk)*p(:,:,n)/landa;
       theta(:,n)=theta(:,n) + q(:,n) * (yk(n) - xk *  theta(:,n));
    end

   kk = [kk;real_data(m,1) theta(1,1)];
   ykk = [ykk ;yk'];
end

%;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;