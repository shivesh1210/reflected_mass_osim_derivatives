%%% Purpose: 
%%%     Recursive computation of gradient of the DRM based on
%%%     solution of the gen. momentum equation using articulated body inertia.
%%%     The robot Jacobian is not needed.
%%%     This employs algorithm 4 with forward recursion 1b)
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%%     j - index of joint w.r.t. which derivative is computed
%%% Last revision: 30 May 2024

function [grad] = Recursive_Grad_DRM(q,h)

global Param;
global Chain;
n = length(q);

lambda0Inv = Recursive_ReflectedMass(q,h);

for i = 1:n
    tmp(i) = Recursive_Diff_ReflectedMass(q,h,i);
end

grad = -tmp/lambda0Inv^2;