function [DD,EE,bb]=TrajectoryConstraints(Dt,Et,bt,N)

% your code goes here
I=eye(N);
DD=kron(I,Dt);
EE=kron(I,Et);
bb=kron(ones(N,1),bt);
end

