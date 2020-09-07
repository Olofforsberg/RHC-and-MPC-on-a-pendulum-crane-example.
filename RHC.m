function K = RHC(H,G,m)
%Receding Horizon Control
% H and G are the cost function matrices
% m is the number of control inputs
% K is the RHC law gain

% your code here
K1=-H\G;
K=K1(1:m,:);
end