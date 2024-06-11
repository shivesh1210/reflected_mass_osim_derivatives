
function [OSIM] = Recursive_InvOSIM_opt(q,h)

global Param;
global Chain;
n = length(Param);

%% Forward recursion %%

% first body is treated separately since there is no predecessor
Chain(1).Crel = SE3Exp(-Param(1).X,q(1))*SE3Inv(Param(1).B);

for i = 2:n
    Chain(i).Crel = SE3Exp(-Param(i).X,q(i))*SE3Inv(Param(i).B);
end

% d = SE3AdjInvMatrix(Chain(n).Cbar*Param(n).A)'*h;
% Chain(n).C = Chain(n).Cbar*Param(n).A;

%% Backward recursion %%

Chain(n).T = eye(6);
Chain(n).MA = Param(n).Mb;   % Articulated Body Inertia
Chain(n).U = Param(n).Mb*Param(n).X;
Chain(n).U2 = SE3AdjMatrix(Chain(n).Crel)'*Chain(n).U;
Chain(n).minv = inv((Param(n).X'*Chain(n).U));
Chain(n).Z = Param(n).X;
for i = n:-1:2
    Chain(i).Ma = Chain(i).MA - Chain(i).minv*Chain(i).U*Chain(i).U';
    Chain(i).S = SE3AdjMatrix(Chain(i).Crel);
    Chain(i-1).MA = Param(i-1).Mb + Chain(i).S'*Chain(i).Ma*Chain(i).S;
    Chain(i-1).U = Chain(i-1).MA*Param(i-1).X;
    Chain(i-1).U2 = SE3AdjMatrix(Chain(i-1).Crel)'*Chain(i-1).U;
    Chain(i-1).minv = inv(Param(i-1).X'*Chain(i-1).U);
    Chain(i-1).T = Chain(i).T*Chain(i).S-Chain(i).minv*Chain(i).Z*Chain(i).U2';
    Chain(i-1).Z = Chain(i-1).T*Param(i-1).X;
end

%% Forward recursion %%
% assuming a stationary robot i.e. V0 = zeros(6,1)
Chain(1).OSIM = Chain(1).minv*Param(1).X*Chain(1).Z';
for i = 2:n
    S = SE3AdjMatrix(Chain(i).Crel);
    Chain(i).D = S*Chain(i-1).OSIM;
    Chain(i).Ubar = Chain(i).D'*Chain(i).U;
    Chain(i).OSIM = Chain(i).D + Chain(i).minv*Param(i).X*(Chain(i).Z - Chain(i).Ubar)';
end
OSIM = Chain(n).OSIM;

% Chain(1).a = Chain(1).minv*Param(1).X*(Chain(1).Z'*h);
% for i = 2:n
%     S = SE3AdjMatrix(Chain(i).Crel);
%     Chain(i).b = S*Chain(i-1).a;
%     Chain(i).a = Chain(i).b + Chain(i).minv*Param(i).X*(Chain(i).Z'*h -Chain(i).b'*Chain(i).U);
% end
% OSIM = Chain(n).a;

end
