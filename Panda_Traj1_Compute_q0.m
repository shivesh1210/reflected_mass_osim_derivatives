%%% Purpose: 
%%%     Compute IK solution of the Panda robot for the pick-place motion
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

% Reflected Mass along y-axis of the EE
h = [0;0;0;0;1;0]; 
h = h/norm(h); % doesn't really have to be a unit vector

% initial value:
q0 = [pi/4,0,0,-pi/2,0,pi/2,0]';

1/Recursive_ReflectedMass(q0,h) % Refl. mass for initial q

[Jd,Cd] = ForwardKinRecursive(q0)

x0 = 0.3;
y0 = 0.3;
z0 = 0.45;
x1 = 0.3;
y1 = -0.3;
z1 = 0.53;
tau = 0;
x = x0+tau*(x1-x0);
y = y0+tau*(y1-y0);
z = z0+tau*(1-tau)*z1;
Cd = [SO3Exp([1,0,0],pi),[x,y,z]';[0,0,0,1]];

%%% choose between pseudoinverse and DRM-minimizing solution
alpha = 1; % DRM-minimizing solution
% alpha = 0; % PI solution

Accuracy = 1e-10;
minGradNorm = 1e-9;
nIterMax = 300;
[q, nIter] = IK_ReflectedMass(q0,Cd,h,alpha,nIterMax,Accuracy,minGradNorm)
[Jd,Cnew] = ForwardKinRecursive(q);
norm(Cnew-Cd)

1/Recursive_ReflectedMass(q,h) % Refl. mass for optimized q

%%% Visiualize the robot pose
fraEmPa = load('frankaEmikaPanda.mat');
robot = fraEmPa.robot;

% robot=importrobot('frankaEmikaPanda.urdf');
config = homeConfiguration(robot);
for i=1:7
    config(i).JointPosition = q(i);
end
show(robot,config);

%%% Results:
% Initial conf: Pseudoinverse solution
% q = [0.769161514359082,-0.242904464766785,0.016431015487036,-2.588109827428299,0.005528509784796,2.345229237713757,0.781244088586008];
% Initial conf: Refl Mass minimizing solution
% q = [-0.161595584936898,-0.407174478683544,0.953916068650922,-2.569927767421870,0.438143520788080,2.276142235575940,0.456663998238660]';