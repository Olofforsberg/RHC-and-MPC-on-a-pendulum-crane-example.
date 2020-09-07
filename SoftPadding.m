function [Hs,gs,Fs,bs,Js,Ls] = mySoftPadding(H,F,bb,J,L,S,rho,m)
% S = weight for quadratic cost of constraint violations
% rho = scalar weight for 1-norm of constraint violations
% m = number of inputs

% your code here
[N1,n]=size(J);
c=size(S,1);
N=N1/(n+c);
I=eye(N);
Sbar=kron(I,S);
[hn,hm]=size(H);
[sn,sm]=size(Sbar);
Hs=[H zeros(hn,sm);zeros(sn,hm) 2*Sbar];
gs=ones(c*N,1)*rho;

In=eye(c);
On=zeros(m,c);
Ibar=kron(I,[-In;-In;On;On]);
IcN=eye(c*N);
Fs=[F Ibar;zeros(c*N,m*N) -IcN];
bs=[bb;zeros(c*N,1)];
OcN=zeros(c*N,n);
Js=[J;OcN];
Ls=[L;OcN];
end

