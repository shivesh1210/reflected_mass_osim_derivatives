%%% Purpose: 
%%%     3D-Animation of the Panda robot performing the circular motion
%%%     
%%% Last revision: 30 May 2024

fraEmPa = load('data/frankaEmikaPanda.mat');
robot = fraEmPa.robot;

% robot=importrobot('frankaEmikaPanda.urdf');
config = homeConfiguration(robot);

load('results/q_Traj2_PI.txt','-ascii')
load('results/q_Traj2_Grad.txt','-ascii')

figure(20)
r = rateControl(10);
for j=1:length(q_Traj2_PI)
    for i=1:7
        config(i).JointPosition = q_Traj2_PI(j,i);
    end
    figure(20);
    show(robot,config)
    waitfor(r);
end

figure(21)
r = rateControl(10);
for j=1:length(q_Traj2_Grad)
    for i=1:7
        config(i).JointPosition = q_Traj2_Grad(j,i);
    end
    figure(21);
    show(robot,config)
    waitfor(r);
end
