%%% Purpose: 
%%%     Recursive computation of gradient of the DRM based on
%%%     solution of the gen. momentum equation using articulated body inertia.
%%%     The robot Jacobian is provided.
%%%     This employs algorithm 4 with forward recursion 1a)
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     j - index of joint w.r.t. which derivative is computed
%%% Last revision: 30 May 2024

function [grad] = Recursive_Grad_DRM_Jac(q,h,J)

global Param;
global Chain;
n = length(q);

lambda0Inv = Recursive_ReflectedMass_Jac(q,h,J);

for i = 1:n
    tmp(i) = Recursive_Diff_ReflectedMass_Jac(q,h,i,J);
end

grad = -tmp/lambda0Inv^2;
