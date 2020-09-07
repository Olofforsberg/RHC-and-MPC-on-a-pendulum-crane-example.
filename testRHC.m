clear variables
close all

%% Load the parameters
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Declare simulation parameters
Ts=0.15;
N=ceil(1.5/Ts);
T=40;
%xTarget=[0.4 0 0.5 0 0 0 0 0]';
xZero=0;
yZero=0;
x0=[xZero 0 yZero 0 0 0 0 0]';
%signal for square movement
w=10;
x1=0.1;
y1=0.1;
x2=0.5;
y2=0.5;
xtest=rectsignal(x1,y1,x2,y2,w,Ts,T);

% Load the matrices using a solution from the previous assignment
[A,B,C,D] = CraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);
%% Declare penalty matrices and tune them here:
Q=eye(8);
Q(1,1)=5; % weight on X
Q(3,3)=5; % weight on Y
Q(5,5)=2; % weight on theta
Q(7,7)=2; % weight on psi
R=eye(2)*0.01;
P=Q;  
%% Compose prediction matrices for RHC
% your myPrediction function is called here and the variables Gamma and Phi are 
% declared in the workspace. 
[Gamma,Phi]=Prediction(A,B,N);
%% Declare RHC control law
% The linear control law K is declared here. It will be visible to a Simulink 
% constant block and can be used to implement the control law.
% See how this is implemented here: SimscapeCrane_RHC/Controllers
[H,G] = CostMatrices(Gamma,Phi,Q,R,P,N);
K = RHC(H,G,size(B,2));
%% Run the simulations for your controller and the PID controller
% Select controller, Uncomment as required
% controlCase=1; % The RHC 
% controlCase=2; % P(ID)controller
controlCase=1;

% Open the model
SimscapeCrane_RHC; 

switch controlCase
    case 1
        sim('SimscapeCrane_RHC');
        responseRHC.output=GantryCraneOutput;
        responseRHC.input=GantryCraneInput;
    case 2  
        sim('SimscapeCrane_RHC');
        responsePP.output=GantryCraneOutput;
        responsePP.input=GantryCraneInput;
    otherwise
        display('wrong setting for variable controlCase. For PID control set controlCase to 1. For a RHC law set controlCase to 2.')
end

%% visualise the performance:
help GantryResponsePlot
switch controlCase
    case 1
        GantryResponsePlot(responseRHC.output.time,responseRHC.input.signals.values,...
            responseRHC.output.signals.values,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xtest,'RHC performance',zZero);
    case 2  
        GantryResponsePlot(responseRHC.output.time,responseRHC.input.signals.values,...
            responseRHC.output.signals.values,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xtest,'RHC performance',zZero);
end