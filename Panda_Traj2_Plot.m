%%% Purpose: 
%%%     Plot the results for the Panda robot performing the circular motion
%%%     
%%% Last revision: 30 May 2024

clear all
close all
clc

load('results/q_Traj2_PI.txt','-ascii')
load('results/RefMass_Traj2_PI.txt','-ascii')
load('results/nIter_Traj2_PI.txt','-ascii')
load('results/Error_Traj2_PI.txt','-ascii')
load('results/q_Traj2_Grad.txt','-ascii')
load('results/RefMass_Traj2_Grad.txt','-ascii')
load('results/nIter_Traj2_Grad.txt','-ascii')
load('results/Error_Traj2_Grad.txt','-ascii')

figure(1);
plot([RefMass_Traj2_PI;RefMass_Traj2_Grad]');
axis([0 length(RefMass_Traj2_PI) -inf inf]);
axis('auto y');

figure(2);
tiledlayout(2,1)
ax1 = nexttile;
plot(ax1,q_Traj2_PI);
title(ax1,'Pseudo Inverse solution');
axis([0 length(q_Traj2_PI) -3 2.5]);
% axis([0 600 -inf inf]);
% axis('auto y');
ax2 = nexttile;
plot(ax2,q_Traj2_Grad);
title(ax2,'DRM-Minimizing solution');
axis([0 length(q_Traj2_Grad) -3 2.5]);
% axis([0 600 -inf inf]);
% axis('auto y');

figure(3);
p3 = bar(nIter_Traj2_Grad,0.9,'FaceColor',[0.4660 0.6740 0.1880]);
% p3(1).FaceColor = [0.4660 0.6740 0.1880];
hold;
figure(3);
bar(nIter_Traj2_PI,0.9,'FaceColor',[0.9290 0.6940 0.1250]);
hold;
