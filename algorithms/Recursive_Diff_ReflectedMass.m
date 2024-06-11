%%% Purpose: 
%%%     Recursive computation of partial derivatives of the DRM based on
%%%     solution of the gen. momentum equation using articulated body inertia.
%%%     The robot Jacobian is not needed.
%%%     This is Algorithm 4 with forward recursion 1b)
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     j - index of joint w.r.t. which derivative is computed
%%% Last revision: 30 May 2024

function [dlambda_h_inv] = Recursive_Diff_ReflectedMass(q,h,j)

global Param;
global Chain;
n = length(Param);

%% Forward recursion %%
for i = 1:n
    if j<i
        Chain(i).dYbar = SE3adMatrix(Chain(j).Ybar)*Chain(i).Ybar;
    else
        Chain(i).dYbar = zeros(6,1);
    end
end    

Chain(1).dd = -SE3adMatrix(Chain(j).Ybar)'*SE3AdjInvMatrix(Chain(n).C)'*h;

%% Backward recursion %%
Chain(n).dp = Chain(n).dYbar'*Chain(1).d + Chain(n).Ybar'*Chain(1).dd;
dp(n) = Chain(n).dp;
Chain(n).dpstar = Chain(n).dp;
Chain(n).dMstar = zeros(6,6);   % Articulated Body Inertia
Chain(n).dU = zeros(6,1);
Chain(n).dminv = 0;
Chain(n).dPi = Chain(n).minv*Chain(n).dp*Chain(n).U;
Chain(n).dMa = [];

for i = j:n-1
    Chain(i).dMstar = zeros(6,6);
    Chain(i).dU = zeros(6,1);
    Chain(i).dminv = 0;
    Chain(i).dp = 0;
    Chain(i).dpstar = 0;
    Chain(i).dPi = zeros(6,1);
end
for i = j:-1:2
    Chain(i).K = Chain(i).U*Chain(i).dU';
    Chain(i).dMa = Chain(i).dMstar - Chain(i).dminv*Chain(i).U*Chain(i).U' - Chain(i).minv*(Chain(i).K+Chain(i).K');
    Chain(i-1).dMstar = Chain(i).Crel_Adj'*Chain(i).dMa*Chain(i).Crel_Adj;
    if j == i
        Chain(i).L = -Chain(i).Ma2*SE3adMatrix(Param(i).Xbar);
        Chain(i-1).dMstar = Chain(i-1).dMstar + Chain(i).L + Chain(i).L';
    end
    Chain(i-1).dU = Chain(i-1).dMstar*Param(i-1).X;
    Chain(i-1).dminv = -Chain(i-1).minv^2*(Param(i-1).X'*Chain(i-1).dU);
    Chain(i-1).dp = Chain(i-1).dYbar'*Chain(1).d + Chain(i-1).Ybar'*Chain(1).dd;
    dp(i-1) = Chain(i-1).dp;
    if j == i
        Chain(i).dPi2 = Chain(i).Crel_Adj'*(Chain(i).dPi - SE3adMatrix(Param(i).X)'*Chain(i).Pi);
    else
        Chain(i).dPi2 = Chain(i).Crel_Adj'*Chain(i).dPi;
    end
    Chain(i-1).dpstar = Chain(i-1).dp - Param(i-1).X'*Chain(i).dPi2;
    Chain(i-1).dPi = Chain(i-1).dminv*Chain(i-1).pstar*Chain(i-1).U ...
        + Chain(i-1).minv*Chain(i-1).dpstar*Chain(i-1).U ...
        + Chain(i-1).minv*Chain(i-1).pstar*Chain(i-1).dU ...
        + Chain(i).dPi2;
end

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
Chain(1).dv = Chain(1).dminv*Chain(1).pstar + Chain(1).minv*Chain(1).dpstar;
dv(1) = Chain(1).dv;
v(1) = Chain(1).v;
p(1) = Chain(1).p;
dp(1) = Chain(1).dp;
Chain(1).dV = Param(1).X*dv(1);
for i = 2:n
    if i == j
        Chain(i-1).dV2 = Chain(i).Crel_Adj*(Chain(i-1).dV - SE3adMatrix(Param(i).Xbar)*Chain(i-1).V);
    else
        Chain(i-1).dV2 = Chain(i).Crel_Adj*Chain(i-1).dV;
    end
    Chain(i).dv = Chain(i).dminv*(Chain(i).pstar - Chain(i).U'*Chain(i-1).V2) + ...
        Chain(i).minv*(Chain(i).dpstar - Chain(i).dU'*Chain(i-1).V2 - Chain(i).U'*Chain(i-1).dV2);
    dv(i) = Chain(i).dv;
    v(i) = Chain(i).v;
    p(i) = Chain(i).p;
    dp(i) = Chain(i).dp;
    Chain(i).dV = Chain(i-1).dV2 + Param(i).X*Chain(i).dv;
end

dlambda_h_inv = p*dv' + dp*v';

end
