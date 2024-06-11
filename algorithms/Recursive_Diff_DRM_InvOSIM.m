%%% Purpose: 
%%%     Recursive computation of partial derivative of the DRM implicitly based on the OSIM 
%%%     This is Algorithm 2
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     j - index of joint w.r.t. which derivative is computed
%%% Last revision: 30 May 2024

%%% Version 2 %%%

function [dDRM] = Recursive_Diff_DRM_InvOSIM(q,h,j)

global Param;
global Chain;
n = length(Param);

%% Backward recursion %%
for i = j:n
    Chain(i).dMA = zeros(6,6);
    Chain(i).dminv = 0;
    Chain(i).dP = zeros(6,6);
    Chain(i).dQ = zeros(6,6);
    Chain(i).da = zeros(6,1);
%     Chain(i).dNA = zeros(6,6);
%     Chain(i).dU = zeros(6,1);
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
    Chain(i-1).dU = Chain(i-1).dMA*Param(i-1).X;
    Chain(i-1).dminv = -Chain(i-1).minv^2*(Param(i-1).X'*Chain(i-1).dU);
    Chain(i-1).da = Chain(i).dQ*Chain(i).a + Chain(i).Q*Chain(i).da;
end

%% Forward recursion %%
Chain(1).dP = Param(1).X*Chain(1).dminv*Param(1).X';
Chain(1).db = Chain(1).dP*Chain(1).a + Chain(1).P*Chain(1).da;
for i = 2:n
    Chain(i).db = Chain(i).dQ'*Chain(i-1).b + Chain(i).Q'*Chain(i-1).db + Chain(i).dP*Chain(i).a + Chain(i).P*Chain(i).da;
end

dDRM = h'*Chain(n).db;

end
