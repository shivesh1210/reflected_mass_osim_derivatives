%%% Purpose: 
%%%     Recursive computation of gradientof the DRM implicitly based on the OSIM 
%%%     This employs algorithm 2
%%% Inputs: 
%%%     q - Joint coordinate vector
%%%     h - direction vector in which the DRM is computed
%%% Last revision: 30 May 2024

function [grad] = Recursive_Grad_DRM_InvOSIM(q,h)

global Param;
global Chain;
n = length(q);

lambda0Inv = Recursive_DRM_InvOSIM(q,h);

for i = 1:n
    tmp(i) = Recursive_Diff_DRM_InvOSIM(q,h,i);
end

grad = -tmp/lambda0Inv^2;