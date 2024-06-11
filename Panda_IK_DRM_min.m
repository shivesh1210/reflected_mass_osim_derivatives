%%% Calculate a DRM-minimizing solution starting from an inital pose with q0 = [pi/4,0,-pi/4,-pi/2,0,pi/2,0]
%%% Two cases are considered:
%%%     1) Minimizing DRM in x-direction
%%%     2) Minimizing DRM in y-direction

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

%%% 1) IK Solution minimizing DRM along x-axis
% h = [0;0;0;1;0;0]; 

%%% 2) IK Solution minimizing DRM along y-axis
h = [0;0;0;0;1;0]; 

% initial value:
q0 = [pi/4,0,-pi/4,-pi/2,0,pi/2,0]';

1/Recursive_ReflectedMass(q0,h) % Refl. mass for initial q

[Jd,Cd] = ForwardKinRecursive(q0);

% set alhpa=0 to get the pseudoinverse solution
alpha = 0.2;
Accuracy = 1e-10;
minGradNorm = 1e-8;
nIterMax = 300;
[qd, nIter] = IK_ReflectedMass(q0,Cd,h,alpha,nIterMax,Accuracy,minGradNorm);
[Jd,Cnew] = ForwardKinRecursive(qd);
norm(Cnew-Cd)

1/Recursive_ReflectedMass(qd,h) % Refl. mass for optimized q

fraEmPa = load('data/frankaEmikaPanda.mat');
robot = fraEmPa.robot;

% robot=importrobot('data/frankaEmikaPanda.urdf');
config = homeConfiguration(robot);
for i=1:7
    config(i).JointPosition = qd(i);
end
show(robot,config);
