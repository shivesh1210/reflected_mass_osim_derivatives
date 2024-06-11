%%% Purpose: 
%%%     Recursive computation of partial derivatives of the DRM based on
%%%     solution of the gen. momentum equation using articulated body inertia.
%%%     The robot Jacobian is provided.
%%%     This is Algorithm 4 with forward recursion 1a)
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     j - index of joint w.r.t. which derivative is computed
%%%     J - FK-Jacobian of the robot
%%% Last revision: 30 May 2024

function [dlambda_h_inv] = Recursive_Diff_ReflectedMass_Jac(q,h,j,J)

global Param;
global Chain;
n = length(Param);

%% Forward recursion %%
for i = 1:j-1
    Chain(i).dp = J(:,j)'*(SE3adMatrix(J(:,i))'*h);
    dp(i) = Chain(i).dp;
end    

%% Backward recursion %%
Chain(n).dp = 0;
dp(n) = 0;
Chain(n).dpA = Chain(n).dp;
Chain(n).dMA = zeros(6,6);   % Articulated Body Inertia
Chain(n).dU = zeros(6,1);
Chain(n).dminv = 0;
Chain(n).dPiA = Chain(n).minv*Chain(n).dp*Chain(n).U;
Chain(n).dMa = [];

for i = j:n-1
    Chain(i).dMA = zeros(6,6);
    Chain(i).dU = zeros(6,1);
    Chain(i).dminv = 0;
    Chain(i).dp = 0;
    dp(i) = 0;
    Chain(i).dpA = 0;
    Chain(i).dPiA = zeros(6,1);
end
for i = j:-1:2
    Chain(i).K = Chain(i).U*Chain(i).dU';
    Chain(i).dMa = Chain(i).dMA - Chain(i).dminv*Chain(i).U*Chain(i).U' - Chain(i).minv*(Chain(i).K+Chain(i).K');
    Chain(i-1).dMA = Chain(i).S'*Chain(i).dMa*Chain(i).S;
    if j == i
        Chain(i).L = -Chain(i).Ma2*SE3adMatrix(Param(i).Xbar);
        Chain(i-1).dMA = Chain(i-1).dMA + Chain(i).L + Chain(i).L';
    end
    Chain(i-1).dU = Chain(i-1).dMA*Param(i-1).X;
    Chain(i-1).dminv = -Chain(i-1).minv^2*(Param(i-1).X'*Chain(i-1).dU);
    if j == i
        Chain(i).dPi2 = Chain(i).S'*(Chain(i).dPiA - SE3adMatrix(Param(i).X)'*Chain(i).PiA);
    else
        Chain(i).dPi2 = Chain(i).S'*Chain(i).dPiA;
    end
    Chain(i-1).dpA = Chain(i-1).dp - Param(i-1).X'*Chain(i).dPi2;
    Chain(i-1).dPiA = Chain(i-1).dminv*Chain(i-1).pA*Chain(i-1).U ...
        + Chain(i-1).minv*Chain(i-1).dpA*Chain(i-1).U ...
        + Chain(i-1).minv*Chain(i-1).pA*Chain(i-1).dU ...
        + Chain(i).dPi2;
end

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
Chain(1).dv = Chain(1).dminv*Chain(1).pA + Chain(1).minv*Chain(1).dpA;
dv(1) = Chain(1).dv;
v(1) = Chain(1).v;
p(1) = Chain(1).p;
Chain(1).dV = Param(1).X*dv(1);
for i = 2:n
    if i == j
        Chain(i-1).dV2 = Chain(i).S*(Chain(i-1).dV - SE3adMatrix(Param(i).Xbar)*Chain(i-1).V);
    else
        Chain(i-1).dV2 = Chain(i).S*Chain(i-1).dV;
    end
    Chain(i).dv = Chain(i).dminv*(Chain(i).pA - Chain(i).U'*Chain(i-1).V2) + ...
        Chain(i).minv*(Chain(i).dpA - Chain(i).dU'*Chain(i-1).V2 - Chain(i).U'*Chain(i-1).dV2);
    dv(i) = Chain(i).dv;
    v(i) = Chain(i).v;
    p(i) = Chain(i).p;
    Chain(i).dV = Chain(i-1).dV2 + Param(i).X*Chain(i).dv;
end

dlambda_h_inv = p*dv' + dp*v';

end
