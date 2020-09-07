function [A,B,C,D] = CraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts)
% Inputs:
% m = Pendulum mass (kg)
% M = Cart mass (kg)
% MR = Rail mass (kg)
% r = String length (m)
% g = gravitational acceleration (m*s^-2)
% Tx = Damping coefficient in X direction (N*s*m^-1)
% Ty = Damping coefficient in Y direction (N*s*m^-1)
% Vm = Input multiplier (scalar)
% Ts = Sample time of the discrete-time system (s)
% Outputs:
% A,B,C,D = State Space matrices of a discrete-time or continuous-time state space model

% The motors in use on the gantry crane are identical and therefore Vx=Vy=Vm.
Vx=Vm;
Vy=Vm;

%A
A=zeros(8);
A(1,2)=1;
A(2,2)=-Tx/(M+MR); A(2,5)=g*m/(M+MR);
A(3,4)=1;
A(4,4)=-Ty/(M); A(4,7)=g*m/(M);
A(5,6)=1;
A(6,2)=Tx/(r*(M+MR)); A(6,5)=-(g*(M+MR+m))/(r*(M+MR));
A(7,8)=1;
A(8,4)=Ty/(M*r); A(8,7)=-(g*(M+m))/(M*r);

%B
B=zeros(8,2);
B(2,1)=Vx/(M+MR);
B(4,2)=Vy/(M);
B(6,1)=-Vx/(r*(M+MR));
B(8,2)=-Vy/(M*r);

%C and D
C=eye(8,8);
D=zeros(8,2);
% if Ts>0 then sample the model, otherwise return a continuous-time model
% with a zero-order hold (piecewise constant) input
if Ts>0
%     Return discrete-time SS model matrices
    H = ss(A,B,C,D);
    Disk=c2d(H,Ts);
    [A,B,C,D]=ssdata(Disk);
end
end