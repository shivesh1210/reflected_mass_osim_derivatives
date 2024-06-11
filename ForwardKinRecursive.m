
function [Jee,Cee] = ForwardKinRecursive(q)

global Param;
global Chain;
n = length(q);

if (n ~= 7) 
    disp('Jacobian is only computed for n=7. Change code if needed!');
    return;    
end

%% Forward recursion %%
% first body is treated separately since there is no predecessor
Chain(1).Crel = SE3Exp(-Param(1).X,q(1))*SE3Inv(Param(1).B);
Chain(1).Cbar = SE3Exp(Param(1).Y,q(1));   
Chain(1).Ybar = SE3AdjMatrix(Chain(1).Cbar)*Param(1).Y;
for i = 2:n
    Chain(i).Crel = SE3Exp(-Param(i).X,q(i))*SE3Inv(Param(i).B);
    Chain(i).Cbar = Chain(i-1).Cbar*SE3Exp(Param(i).Y,q(i));   
    Chain(i).Ybar = SE3AdjMatrix(Chain(i).Cbar)*Param(i).Y;
end
Chain(n).C = Chain(n).Cbar*Param(n).A;

Cee = Chain(n).C;
Jee = SE3AdjInvMatrix(Cee)*[Chain(1).Ybar, Chain(2).Ybar, Chain(3).Ybar, Chain(4).Ybar, Chain(5).Ybar, Chain(6).Ybar, Chain(7).Ybar];   
