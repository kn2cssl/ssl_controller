%defining constants

syms n R km r J Jw Je d m a1 a2 a3 a4 Bx By Bt
g1=n*km*Jw/(R*r*Je);
g2=n*km/(R*r);
h1=g1*km*n/m;
h2=d*g1*km*n/J;
h3=km*n*g2;
h4=d*g1/J;
%defining state matrixes
A=[-Bx 0 0 h1*sin(a1) h1*sin(a2) h1*sin(a3) h1*sin(a4)
    0 -By 0 -h1*cos(a1) -h1*cos(a2) -h1*cos(a3) -h1*cos(a4)
    0 0 -Bt -h2 -h2 -h2 -h2
    0 0 0 -h3 0 0 0
    0 0 0 0 -h3 0 0
    0 0 0 0 0 -h3 0
    0 0 0 0 0 0 -h3];
inv(A)