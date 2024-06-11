%%% Purpose: 
%%%     Recursive computation of the DRM based on solution of the 
%%%     gen. momentum equation using articulated body inertia.
%%%     The robot Jacobian is not needed.
%%%     This is Algorithm 3 with forward recursion 1b)
%%% Inputs: 
%%%      q - Joint coordinate vector
%%%      h - direction vector in which the DRM is computed
%%% Last revision: 30 May 2024

function [lambda_h_inv] = Recursive_ReflectedMass(q,h)

global Param;
global Chain;
n = length(Param);

%% Forward recursion %%
% first body is treated separately since there is no predecessor
Chain(1).Crel = SE3Exp(-Param(1).X,q(1))*SE3Inv(Param(1).B);
Chain(1).Cbar = SE3Exp(Param(1).Y,q(1));   
Chain(1).Ybar = SE3AdjMatrix(Chain(1).Cbar)*Param(1).Y;
for i = 2:n
    Chain(i).Crel = SE3Exp(-Param(i).X,q(i))*SE3Inv(Param(i).B);
    Chain(i).Cbar = Chain(i-1).Cbar*SE3Exp(Param(i).Y,q(i));   
    Chain(i).Ybar = SE3AdjMatrix(Chain(i).Cbar)*Param(i).Y;
end
Chain(n).C = Chain(n).Cbar*Param(n).A;
Chain(1).d = SE3AdjInvMatrix(Chain(n).C)'*h; % d is stored at the first element, just to have it somewhere

%% Backward recursion %%
Chain(n).p = Chain(n).Ybar'*Chain(1).d;
p(n) = Chain(n).p;
Chain(n).pstar = Chain(n).p;
Chain(n).Mstar = Param(n).Mb;   % Articulated Body Inertia
Chain(n).U = Param(n).Mb* Param(n).X;
Chain(n).minv = inv((Param(n).X'*Chain(n).U));
Chain(n).Pi = Chain(n).minv*Chain(n).p*Chain(n).U;
for i = n:-1:2
    Chain(i).Ma = Chain(i).Mstar - Chain(i).minv*Chain(i).U*Chain(i).U';
    Chain(i).Crel_Adj = SE3AdjMatrix(Chain(i).Crel);
    Chain(i).Ma2 = Chain(i).Crel_Adj'*Chain(i).Ma*Chain(i).Crel_Adj;
    Chain(i-1).Mstar = Param(i-1).Mb + Chain(i).Ma2;
    Chain(i-1).U = Chain(i-1).Mstar*Param(i-1).X;
    Chain(i-1).minv = inv(Param(i-1).X'*Chain(i-1).U);
    Chain(i-1).p = Chain(i-1).Ybar'*Chain(1).d;
    p(i-1) = Chain(i-1).p;
    Chain(i).Pi2 = Chain(i).Crel_Adj'*Chain(i).Pi;
    Chain(i-1).pstar = Chain(i-1).p - Param(i-1).X'*Chain(i).Pi2;
    Chain(i-1).Pi = Chain(i-1).minv*Chain(i-1).pstar*Chain(i-1).U+Chain(i).Pi2;
end

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
Chain(1).v = Chain(1).minv*Chain(1).pstar;
v(1) = Chain(1).v;
Chain(1).V = Param(1).X*Chain(1).v;
for i = 2:n
    Chain(i-1).V2 = Chain(i).Crel_Adj*Chain(i-1).V;
    Chain(i).v = Chain(i).minv*(Chain(i).pstar - Chain(i).U'*Chain(i-1).V2);
    v(i) = Chain(i).v;
    Chain(i).V = Chain(i-1).V2 + Param(i).X*Chain(i).v;
end

lambda_h_inv = p*v';

end
