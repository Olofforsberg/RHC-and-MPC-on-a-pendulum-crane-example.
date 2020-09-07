clear variables
close all
clc
%% SimscapeCrane_MPC_start;
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Set sample period and prediction horizon
Ts=1.5/10;
Tf=1.5; % duration of prediction horizon in seconds
N=ceil(Tf/Ts); % ceiling to ensure horizon length N is an integer

%% Load the dynamics matrices using a solution from last assignment
[A,B,C,~] = CraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);

%% Define other simulation parameters
T=40; % duration of simulation
%xTarget=0.8*[xRange(2) 0 yRange(2) 0 0 0 0 0]'; % target equilibrium state
xZero=xRange(2)*0.05; yZero=yRange(2)*0.05;
x0=[xZero 0 yZero 0 0 0 0 0]'; % initial state

%signal for square movement
w=10;
x1=xZero;
y1=yZero;
x2=0.8*xRange(2);
y2=0.8*yRange(2);
xtest=rectsignal(x1,y1,x2,y2,w,Ts,T);
%xtest=rampsignal(x1,y1,x2,y2,Ts,T);



%% Declare penalty matrices and tune them here:
Q=zeros(8);
Q(1,1)=5; % weight on X
Q(3,3)=5; % weight on Y
% In order to test constraints, do not penalise angle or derivative of
% angle. Instead impose soft constraints on them. 
Q(5,5)=2; % weight on theta
Q(7,7)=2; % weight on psi
R=eye(2)*0.01; % very small penalty on input to demonstrate hard constraints
P=Q; % terminal weight

%% Declare contraints
% Constrain only states (X,Y,theta,psi)
% Constrained vector is Dx, hence
D=zeros(4,8);D(1,1)=1;D(2,3)=1;D(3,5)=1;D(4,7)=1;

angleConstraint=7*pi/180; % in radians
cl=[0;  0; -angleConstraint;  -angleConstraint];
ch=[0.9*xRange(2);  0.9*yRange(2);  angleConstraint;  angleConstraint];

% Input constraints (hard)
ul=[-1; -1];
uh=[1; 1];

%% Compute stage constraint matrices and vector
[Dt,Et,bt]=StageConstraints(A,B,D,cl,ch,ul,uh);

%% Compute trajectory constraints matrices and vector
[DD,EE,bb]=TrajectoryConstraints(Dt,Et,bt,N);

%% Compute QP constraint matrices
[Gamma,Phi] = Prediction(A,B,N); % get prediction matrices:
[F,J,L] = ConstraintMatrices(DD,EE,Gamma,Phi,N);

%% Compute QP cost matrices
[H,G] = CostMatrices(Gamma,Phi,Q,R,P,N);

%% Compute matrices and vectors for soft constraints
% Define weights for constraint violations
rho = 1e3; % weight for exact penalty term
S = 1e-3*eye(size(D,1)); % small positive definite quadratic cost to ensure uniqueness
[Hs,gs,Fs,bs,Js,Ls] = SoftPadding(H,F,bb,J,L,S,rho,size(B,2));

%% replace matrices and vectors to simplify code 
H = Hs;
F = Fs;
bb = bs;
J = Js;
L = Ls;

%% Prepare cost and constraint matrices for mpcqpsolver
% Calculating the inverse of the lower triangular H. see doc mpcqpsolver.
% It is done here rather than inside myMPController for speed and generality
[H,p] = chol(H,'lower');
H=H\eye(size(H));

%% Running a Matlab simulation and visualising the results:
MatlabSimulation
GantryResponsePlot(t,allU,x,ul,uh,cl,ch,[1 3 5 7],xtest,'Linear simulation',zZero);

%% Run the Simulink simulation for your controller
% Note that in order to test your controller you have to navigate to
% SimscapeCrane_MPCsoft/MPC to copy and paste your controller code inside the
% Matlab Function block

%SimscapeCrane_MPCsoft3
SimscapeCrane_MPChard
sim('SimscapeCrane_MPCsoft3');
responseRHC.output=GantryCraneOutput;
responseRHC.input=GantryCraneInput;
%% visualise the performance:
GantryResponsePlot(responseRHC.output.time',responseRHC.input.signals.values,...
    responseRHC.output.signals.values,ul,uh,cl,ch,[1 3 5 7],xtest,'Nonlinear simulation',zZero);

%%position of the mass