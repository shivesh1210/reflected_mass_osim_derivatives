%%% Purpose: 
%%%     Compute IK solution of the Panda robot for the circular motion
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

Panda_SetParam;

% Reflected Mass along the x-axis of the EE
h = [0;0;0;1;0;0]; 
h = h/norm(h); % doesn't really have to be a unit vector

% initial value:
q0 = [pi/4,0,0,-pi/2,0,pi/2,0]';

1/Recursive_ReflectedMass(q0,h) % Refl. mass for initial q0

[Jd,Cd] = ForwardKinRecursive(q0)

x0 = 0.4;
y0 = 0.4;
z0 = 0.45;
R = 0.2;
tau = 0;
x = x0+R*cos(2*pi*tau+pi/2);
y = y0+R*sin(2*pi*tau+pi/2);
z = z0;
Cd = [SO3Exp([1,0,0],pi),[x,y,z]';[0,0,0,1]];
    
%%% choose between pseudoinverse and DRM-minimizing solution
alpha = 1; % DRM-minimizing solution
% alpha = 0; % PI solution

Accuracy = 1e-7;
minGradNorm = 1e-6;
nIterMax = 300;
[q, nIter] = IK_ReflectedMass(q0,Cd,h,alpha,nIterMax,Accuracy,minGradNorm)
[Jd,Cnew] = ForwardKinRecursive(q);
norm(Cnew-Cd)

1/Recursive_ReflectedMass(q,h) % Refl. mass for optimized q0

%%% Visiualize the robot pose
fraEmPa = load('data/frankaEmikaPanda.mat');
robot = fraEmPa.robot;
% robot = importrobot('frankaEmikaPanda.urdf');

% robot=importrobot('frankaEmikaPanda.urdf');
config = homeConfiguration(robot);
for i=1:7
    config(i).JointPosition = q(i);
end
show(robot,config);


%%% Results:
% Initial conf: Pseudoinverse solution
% q = [0.917159398739465,0.623897461011205,0.087321656612431,-1.395653765011827,-0.056511765189412,2.017119953889014,1.012506386810966]';
% Initial conf: Refl Mass minimizing solution
% q = [0.223584127007922,1.061293167511053,1.250048915466862,-1.334275408540202,-0.994289478183618,1.724683086618973,1.427948597976861]';
