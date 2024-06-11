%%% Purpose: Recursive computation of the inverse OSIM 
%%%          This is Algorithm 5
%%% Inputs: 
%%%      q - Joint coordinate vector
%%%      h - direction vector in which the DRM is computed
%%% Last revision: 30 May 2024

function [OSIM] = Recursive_InvOSIM(q,h)

global Param;
global Chain;
n = length(Param);

for i = 2:n
    Chain(i).Crel = SE3Exp(-Param(i).X,q(i))*SE3Inv(Param(i).B);
    Chain(i).S = SE3AdjMatrix(Chain(i).Crel);
end

%% Backward recursion %%
Chain(n).T = eye(6);
Chain(n).MA = Param(n).Mb;   % Articulated Body Inertia
Chain(n).U = Param(n).Mb* Param(n).X;
Chain(n).minv = inv((Param(n).X'*Chain(n).U));
for i = n:-1:2
    Chain(i).P = Chain(i).minv*Param(i).X*Param(i).X';
    Chain(i).NA = eye(6) - Chain(i).MA*Chain(i).P;
    Chain(i).Q = Chain(i).S'*Chain(i).NA;
    Chain(i-1).MA = Param(i-1).Mb + Chain(i).Q*Chain(i).MA*Chain(i).S;
    Chain(i-1).T = Chain(i).T*Chain(i).NA'*Chain(i).S;
    Chain(i-1).U = Chain(i-1).MA*Param(i-1).X;
    Chain(i-1).minv = inv(Param(i-1).X'*Chain(i-1).U);
end

%% Forward recursion %%
Chain(1).P = Param(1).X*Chain(1).minv*Param(1).X';
Chain(1).NA = eye(6) - Chain(1).MA*Chain(1).P;
Chain(1).OSIM = Chain(1).P*Chain(1).T';
for i = 2:n
    Chain(i).OSIM = Chain(i).Q'*Chain(i-1).OSIM + Chain(i).P*Chain(i).T';
end

OSIM = Chain(n).OSIM;

end
