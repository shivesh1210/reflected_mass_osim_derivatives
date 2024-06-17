%%% Purpose: 
%%%     Compute trajectory of the Panda robot for the circular motion
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

saveResult = 0; % set to 1 if the results should be saved

Panda_SetParam;

% Reflected Mass along the x-axis of the EE
h = [0;0;0;1;0;0];
h = h/norm(h); % doesn't really have to be a unit vector

Accuracy = 1e-7;
alpha = 0.8;
minGradNorm = 1e-1;
nIterMax = 60; % large number - just to be conservative

% Circular motion
x0 = 0.4;
y0 = 0.4;
z0 = 0.45;
R = 0.2;
T = 6;
dt = 0.002;
nSteps = T/dt;

% Initial values computed with Panda_Traj2_Compute_q0.m
if alpha == 0
% Initial conf: Pseudoinverse solution
q0 = [0.917159398739465,0.623897461011205,0.087321656612431,-1.395653765011827,-0.056511765189412,2.017119953889014,1.012506386810966]';
else
% Initial conf: Refl Mass minimizing solution
q0 = [0.223584127007922,1.061293167511053,1.250048915466862,-1.334275408540202,-0.994289478183618,1.724683086618973,1.427948597976861]';
end

qTraj = zeros([nSteps+1 7]);
qTraj(1,:) = q0;
nIter = zeros([1,nSteps+1]);
RefMassTraj = zeros([1,nSteps+1]);
RefMassTraj(1) = 1/Recursive_ReflectedMass(q0,h);
ErrorTraj = zeros([1,nSteps+1]);

[J0,C0] = ForwardKinRecursive(q0);

for i=1:nSteps
    tau = (i)*dt;
    x = x0+R*cos(pi*tau+pi/2);
    y = y0+R*sin(pi*tau+pi/2);
    z = z0;
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

if saveResult == 1
    if alpha == 0
        save('results/q_Traj2_PI.txt','qTraj','-ascii')
        save('results/RefMass_Traj2_PI.txt','RefMassTraj','-ascii')
        save('results/nIter_Traj2_PI.txt','nIter','-ascii')
        save('results/Error_Traj2_PI.txt','ErrorTraj','-ascii')
    else
        save('results/q_Traj2_Grad.txt','qTraj','-ascii')
        save('results/RefMass_Traj2_Grad.txt','RefMassTraj','-ascii')
        save('results/nIter_Traj2_Grad.txt','nIter','-ascii')
        save('results/Error_Traj2_Grad.txt','ErrorTraj','-ascii')
    end
end

return

robot=importrobot('data/frankaEmikaPanda.urdf');
config = homeConfiguration(robot);
% for i=1:7
%     config(i).JointPosition = q0(i);
% end
% figure(4+figOffset);
% show(robot,config)

r = rateControl(10);
for j=1:length(qTraj)
    for i=1:7
        config(i).JointPosition = qTraj(j,i);
    end
    figure(5+figOffset);
    show(robot,config)
    waitfor(r);
end

% for i=1:7
%         config(i).JointPosition = qTraj(2,i);
% end
% figure(6+figOffset);
% show(robot,config)

