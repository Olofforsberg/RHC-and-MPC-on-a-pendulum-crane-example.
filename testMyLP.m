clear variables
close all
clc
%% SimscapeCrane_MPC_start;
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Load the dynamics matrices using a solution from last assignment
Ts=0.15;
[A,B,C,D] = myCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);

%% Define other parameters
N=ceil(1.5/Ts); % ceiling to ensure N is an integer
T=40;
xTarget=[xRange(2)*0.8 0 xRange(2)*0.8 0 0 0 0 0]';% target equilibrium state
x0=[0.1 0 0.1 0 0 0 0 0]'; % starting offset

w=10;
x1=0.1;
y1=0.1;
x2=0.4;%0.8*xRange(2);
y2=0.4;%0.8*yRange(2);
xtest=rectsignal(x1,y1,x2,y2,w,Ts,T);
%% Declare penalty matrices and tune them here:
Q=eye(8);
Q(1,1)=5; % weight on X
Q(3,3)=5; % weight on Y
% In order to test constraints, do not penalise angle or derivative of
% angle. Instead impose soft constraints on them. 
Q(5,5)=2; % weight on theta
Q(7,7)=2; % weight on psi
R=eye(2)*0.01; % very small penalty on input to demonstrate hard constraints
P=Q; % terminal weight

%% Declare contraints
% Declaring constraints only on states (X,Y,theta,psi) and inputs u
angleConstraint=7*pi/180; % in radians
cl=[0;  0; -angleConstraint;  -angleConstraint];
ch=[0.9*xRange(2);  0.9*yRange(2);  angleConstraint;  angleConstraint];
ul=[-1; -1];
uh=[1; 1];
% constrained vector is Dx, hence
D=zeros(4,8);D(1,1)=1;D(2,3)=1;D(3,5)=1;D(4,7)=1;

%% Compute stage constraint matrices and vector
[Dt,Et,bt]=myStageConstraints(A,B,D,cl,ch,ul,uh);

%% Compute trajectory constraints matrices and vector
[DD,EE,bb]=myTrajectoryConstraints(Dt,Et,bt,N);

%% Compute QP constraint matrices
[Gamma,Phi] = myPrediction(A,B,N); % get prediction matrices:
[F,J,L]=myConstraintMatrices(DD,EE,Gamma,Phi,N);


%% Compute QP cost matrices
[Al,blx0,blxe,ucon,Dx0,CD] = myCostMatrices2(Gamma,Phi,Q,R,P,N,ul,uh,D,A,B,cl,ch);

%%LP solver
%[ft,Al,JL,Ll]=LPmatrices(Gamma,Phi,F,bb,J);
%linprog

[utest,flag]=myLPController(Al,blx0,blxe,ucon,x0,xTarget,2,N,Dx0,CD)
%% Prepare cost and constraint matrices for mpcqpsolver
% Calculating the inverse of the lower triangular H. see doc mpcqpsolver.


%% Running a matlab simulation and visualisng the results:
MatlabSimulation2
GantryResponsePlot(t,allU,...
    x,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xtest,'Linear simulation: MPC performance',zZero);
%% Run the Simulink simulation for your controller
% Note that in order to test your controller you have to navigate to
% SimscapeCrane_MPChard/MPC and copy paste your controller code inside the
% Matlab Function block

% SimscapeCrane_MPChard
% sim('SimscapeCrane_MPChard');
% responseRHC.output=GantryCraneOutput;
% responseRHC.input=GantryCraneInput;
% %% visualise the performance:
% GantryResponsePlot(responseRHC.output.time,responseRHC.input.signals.values,...
%     responseRHC.output.signals.values,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],...
%     [1 3],xTarget,'Nonlinear simulation: MPC performance',zZero);