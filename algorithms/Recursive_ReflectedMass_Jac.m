%%% Purpose: 
%%%     Recursive computation of the DRM based on solution of the 
%%%     gen. momentum equation using articulated body inertia.
%%%     The robot Jacobian is provided.
%%%     This is Algorithm 3 with forward recursion 1a)
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     J - FK-Jacobian of the robot
%%% Last revision: 30 May 2024


function [lambda_h_inv] = Recursive_ReflectedMass_Jac(q,h,J)

global Param;
global Chain;
n = length(Param);

%% Forward recursion %%
for i = 2:n
    Chain(i).S = SE3AdjMatrix(SE3Exp(-Param(i).X,q(i))*SE3Inv(Param(i).B));
end

p = J'*h;

%% Backward recursion %%
Chain(n).p = p(n);
Chain(n).pA = p(n);
Chain(n).MA = Param(n).Mb;   % Articulated Body Inertia
Chain(n).U = Param(n).Mb*Param(n).X;
Chain(n).minv = inv((Param(n).X'*Chain(n).U));
Chain(n).PiA = Chain(n).minv*Chain(n).p*Chain(n).U;
for i = n:-1:2
    Chain(i).Ma = Chain(i).MA - Chain(i).minv*Chain(i).U*Chain(i).U';
    Chain(i).Ma2 = Chain(i).S'*Chain(i).Ma*Chain(i).S;
    Chain(i-1).MA = Param(i-1).Mb + Chain(i).Ma2;
    Chain(i-1).U = Chain(i-1).MA*Param(i-1).X;
    Chain(i-1).minv = inv(Param(i-1).X'*Chain(i-1).U);
    Chain(i-1).p = p(i-1);
    Chain(i).Pi2 = Chain(i).S'*Chain(i).PiA;
    Chain(i-1).pA = Chain(i-1).p - Param(i-1).X'*Chain(i).Pi2;
    Chain(i-1).PiA = (Chain(i-1).minv*Chain(i-1).pA)*Chain(i-1).U + Chain(i).Pi2;
end

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
v(1) = Chain(1).minv*Chain(1).pA;
Chain(1).v = v(1);
Chain(1).V = Param(1).X*v(1);
for i = 2:n
    Chain(i-1).V2 = Chain(i).S*Chain(i-1).V;
    v(i) = Chain(i).minv*(Chain(i).pA - Chain(i).U'*Chain(i-1).V2);
    Chain(i).v = v(i);
    Chain(i).V = Chain(i-1).V2 + Param(i).X*v(i);
end

lambda_h_inv = p'*v';

end
