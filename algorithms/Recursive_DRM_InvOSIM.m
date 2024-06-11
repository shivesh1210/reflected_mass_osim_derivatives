%%% Purpose: 
%%%     Recursive computation of the DRM implicitly based on the OSIM 
%%%     This is Algorithm 1
%%% Inputs: 
%%%      q - Joint coordinate vector
%%%      h - direction vector in which the DRM is computed
%%% Last revision: 30 May 2024

function [DRM] = Recursive_DRM_InvOSIM(q,h)

global Param;
global Chain;
n = length(Param);

%% Forward recursion %%
for i = 2:n
    Chain(i).Crel = SE3Exp(-Param(i).X,q(i))*SE3Inv(Param(i).B);
    Chain(i).S = SE3AdjMatrix(Chain(i).Crel);
end

%% Backward recursion %%
Chain(n).a = h;
Chain(n).MA = Param(n).Mb;   % Articulated Body Inertia
for i = n:-1:2
    Chain(i).U = Chain(i).MA*Param(i).X;
    Chain(i).minv = inv(Param(i).X'*Chain(i).U);
    Chain(i).P = Chain(i).minv*Param(i).X*Param(i).X';
    Chain(i).NA = eye(6) - Chain(i).MA*Chain(i).P;
    Chain(i).Q = Chain(i).S'*Chain(i).NA;
    Chain(i-1).MA = Param(i-1).Mb + Chain(i).Q*Chain(i).MA*Chain(i).S;
    Chain(i-1).a = Chain(i).Q*Chain(i).a;
end
Chain(1).U = Chain(1).MA*Param(1).X;
Chain(1).minv = inv(Param(1).X'*Chain(1).U);
Chain(1).P = inv((Param(1).X'*Chain(1).MA*Param(1).X))*Param(1).X*Param(1).X';
Chain(1).NA = eye(6) - Chain(1).MA*Chain(1).P;

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
Chain(1).b = Chain(1).P*Chain(1).a;
for i = 2:n
    Chain(i).b = Chain(i).Q'*Chain(i-1).b + Chain(i).P*Chain(i).a;
end

DRM = h'*Chain(n).b;

end
