function [u,status,iA1] = MPController(H,G,gs,F,bb,J,L,x,xTarget,nu,iA)
% H       - quadratic term in the cost function (Linv if using mpcqpsolver).
% G       - matrix in the linear term of the cost function.
% gs      - vector in the linear term of the cost function.
% F       - LHS of the inequalities term.
% bb      - RHS of the inequalities term.
% J       - RHS of inequalities term.
% L       - RHS of inequalities term. Use this to shift constraints around target point
% x       - current state
% xTarget - target equilibrium point.
% nu      - Number of inputs.
% iA      - active inequalities, see doc mpcqpsolver
%
% u is the first input vector u_0 in the sequence [u_0; u_1; ...; u_{N-1}]; 
% In other words, u is the value of the receding horizon control law
% evaluated with the current state x and target xTarget

% For more info on mpcqpsolver, read the documentation on mpcqpsolver to understand how it is
% suppose to be used. Use iA and iA1 to pass the active inequality vector 
nu=2;
opt = mpcqpsolverOptions;
opt.IntegrityChecks = false; % for code generation
opt.FeasibilityTol = 1e-3;
opt.DataType = 'double';

Gnew=[G*(x-xTarget);gs];
b=bb+J*x+L*xTarget;
[U,status,iA1]=mpcqpsolver(H,Gnew,-F,-b,[],zeros(0,1),iA,opt);
u=U(1:nu);
end

