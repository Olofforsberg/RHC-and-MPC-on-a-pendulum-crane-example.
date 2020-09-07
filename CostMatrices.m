function [H,G] = CostMatrices(Gamma,Phi,Q,R,P,N)
%% cost function matrices
% Gamma and Phi are the prediction matrices
% Q is the stage cost weight on the states, i.e. x'Qx
% R is the stage cost weight on the inputs, i.e. u'Ru
% P is the terminal weight on the final state

[a,b]=size(P);
I=eye(N);
I2=eye(N-1);
Qbar=kron(I2,Q);
Qbar(end+1:end+a,:)=zeros(a,length(Qbar(1,:)));
Qbar(:,end+1:end+b)=zeros(length(Qbar(:,1)),b);
Qbar(end-a+1:end,end-b+1:end)=P;
Rbar=kron(I,R);

H=2*(Rbar+Gamma'*Qbar*Gamma);
G=2*(Gamma'*Qbar*Phi);
end