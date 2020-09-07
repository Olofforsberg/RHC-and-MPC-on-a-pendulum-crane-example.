function [Gamma,Phi] = Prediction(A,B,N)
% MYPREDICTION  [Gamma,Phi] = myPrediction(A,B,N). 
% A and B are discrete-time state space matrices for x[k+1]=Ax[k]+Bu[k]
% N is the horizon length. 
% Your code is suppose to work for any linear system, not just the gantry crane. 

% your code here:
s=size(A);
n=s(2);
I1=eye(N);
I2=zeros(N);
I2(2:end,1:end-1)=eye(N-1);

A1=eye(n*N)-kron(I2,A);
A2=[A;zeros(n*(N-1),n)];
B=kron(I1,B);

Gamma = A1\B;
Phi = A1\A2;
end