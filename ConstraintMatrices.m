function [F,J,L]=ConstraintMatrices(DD,EE,Gamma,Phi,N)

n1 =size(DD,2);
n=n1/N;
m1 =size(EE,2);
m=m1/N;
Gamma2=[zeros(n,N*m);Gamma(1:(N-1)*n,:)];
Phi2=[eye(n);Phi(1:(N-1)*n,:)];
F=DD*Gamma2+EE;
J=-DD*Phi2;

L=-J-DD*kron(ones(N,1),eye(n));
end

