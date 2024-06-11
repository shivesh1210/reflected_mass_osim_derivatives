%%% Purpose: 
%%%     Compute trajectory of the Panda robot for the pick-place motion
%%%     
%%% Remark:
%%%     choose alpha = 0 for the PI solution
%%%     choose alpha = 1 for the DRM-minimizing solution
%%% Last revision: 30 May 2024

clear all
close all
clc

% add paths to helper functions and algorithms
addpath('helper_functions/');
addpath('algorithms/');

global Param; % Structure with all geoemtric and dynamic robot parameters
global Chain; % Structure with all temporal data
global ee;

global n; % DOF, number of joints

saveResult = 0;

Panda_SetParam;

% Reflected Mass along y-axis of the EE
h = [0;0;0;0;1;0];
h = h/norm(h); % doesn't really have to be a unit vector

Accuracy = 1e-7;
alpha = 1;
minGradNorm = 1e-1;
nIterMax = 60; % large number - just to be conservative

% Initial values computed with Panda_Traj1_Compute_q0.m
if alpha == 0
% Initial conf: Pseudoinverse solution
q0 = [0.769161514359082,-0.242904464766785,0.016431015487036,-2.588109827428299,0.005528509784796,2.345229237713757,0.781244088586008]';
else
% Initial conf: Refl Mass minimizing solution
q0 = [-0.161595584936898,-0.407174478683544,0.953916068650922,-2.569927767421870,0.438143520788080,2.276142235575940,0.456663998238660]';
end

% Pick and place path
x0 = 0.3;
y0 = 0.3;
z0 = 0.45;
x1 = 0.3;
y1 = -0.3;
z1 = 0.53;
T = 2;
dt = 0.002;
nSteps = T/dt;

qTraj = zeros([nSteps+1 7]);
qTraj(1,:) = q0;
nIter = zeros([1,nSteps+1]);
RefMassTraj = zeros([1,nSteps+1]);
RefMassTraj(1) = 1/Recursive_ReflectedMass(q0,h);
ErrorTraj = zeros([1,nSteps+1]);

[J0,C0] = ForwardKinRecursive(q0);

for i=1:nSteps
    tau = sin(i*dt*pi/2)^2;
    x = x0+tau*(x1-x0);
    y = y0+tau*(y1-y0);
    z = z0+tau*(1-tau)*z1;
    Cd = [SO3Exp([1,0,0],pi),[x,y,z]';[0,0,0,1]];
    [qTraj(i+1,:),nIter(i+1)] = IK_ReflectedMass(qTraj(i,:)',Cd,h,alpha,nIterMax,Accuracy,minGradNorm);
    RefMassTraj(i+1) = 1/Recursive_ReflectedMass(qTraj(i+1,:),h);
    [Jd,Cnew] = ForwardKinRecursive(qTraj(i+1,:));
    ErrorTraj(i+1) = norm(Cnew-Cd);
end

if alpha == 0
    figOffset = 0;
else
    figOffset = 6;
end
figure(1+figOffset);
plot(qTraj)
figure(2+figOffset);
plot(RefMassTraj)
figure(3+figOffset);
plot(nIter)

mkdir('results'); % create results folder if it does not exist

if saveResult == 0
    if alpha == 0
        save('results/q_Traj1_PI.txt','qTraj','-ascii')
        save('results/RefMass_Traj1_PI.txt','RefMassTraj','-ascii')
        save('results/nIter_Traj1_PI.txt','nIter','-ascii')
        save('results/Error_Traj1_PI.txt','ErrorTraj','-ascii')
    else
        save('results/q_Traj1_Grad.txt','qTraj','-ascii')
        save('results/RefMass_Traj1_Grad.txt','RefMassTraj','-ascii')
        save('results/nIter_Traj1_Grad.txt','nIter','-ascii')
        save('results/Error_Traj1_Grad.txt','ErrorTraj','-ascii')
    end
end

% for i=1:7
%         config(i).JointPosition = qTraj(2,i);
% end
% figure(6+figOffset);
% show(robot,config)

