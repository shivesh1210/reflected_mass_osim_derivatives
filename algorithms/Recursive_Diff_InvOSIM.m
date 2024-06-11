%%% Purpose: 
%%%     Recursive computation of partial derivative of inverse OSIM 
%%%     This is Algorithm 6
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     j - index of joint w.r.t. which derivative is computed
%%% Last revision: 30 May 2024

%%% Version 2 %%%

function [dOSIM] = Recursive_Diff_InvOSIM(q,j)

global Param;
global Chain;
n = length(Param);

%% Backward recursion %%
Chain(n).dT = zeros(6,6);
Chain(n).dMA = zeros(6,6);   % Articulated Body Inertia
Chain(n).dNA = zeros(6,6);
Chain(n).dU = zeros(6,1);
Chain(n).dP = zeros(6,6);
Chain(n).dQ = zeros(6,6);
Chain(n).dminv = 0;
for i = j:n-1
    Chain(i).dMA = zeros(6,6);
    Chain(i).dU = zeros(6,1);
    Chain(i).dminv = 0;
    Chain(i).dT = zeros(6,6);
    Chain(i).dNA = zeros(6,6);
    Chain(i).dP = zeros(6,6);
    Chain(i).dQ = zeros(6,6);
end
for i = j:-1:2
    Chain(i).dP = Param(i).X*Chain(i).dminv*Param(i).X';
    Chain(i).dNA = - Chain(i).dMA*Chain(i).P - Chain(i).MA*Chain(i).dP;
    Chain(i).dQ = Chain(i).S'*Chain(i).dNA;
    if j == i
        Chain(i).dQ = Chain(i).dQ - SE3adMatrix(Param(i).Xbar)'*Chain(i).Q;
    end
    Chain(i-1).dMA = (Chain(i).dQ*Chain(i).MA + Chain(i).Q*Chain(i).dMA)*Chain(i).S;
    if j == i
        Chain(i-1).dMA = Chain(i-1).dMA + (Param(i-1).Mb - Chain(i-1).MA)*SE3adMatrix(Param(i).Xbar);
    end
    Chain(i-1).dT = Chain(i).dT*Chain(i).Q' + Chain(i).T*Chain(i).dQ';
    Chain(i-1).dU = Chain(i-1).dMA*Param(i-1).X;
    Chain(i-1).dminv = -Chain(i-1).minv^2*(Param(i-1).X'*Chain(i-1).dU);
end

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
Chain(1).dP = Param(1).X*Chain(1).dminv*Param(1).X';
Chain(1).dOSIM = Chain(1).dP*Chain(1).T' + Chain(1).P*Chain(1).dT';
for i = 2:n
    Chain(i).dOSIM = Chain(i).dQ'*Chain(i-1).OSIM + Chain(i).Q'*Chain(i-1).dOSIM + Chain(i).dP*Chain(i).T' + Chain(i).P*Chain(i).dT';
end

dOSIM = Chain(n).dOSIM;

end
