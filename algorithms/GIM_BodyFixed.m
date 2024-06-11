%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Shivesh Kumar
% last modified: February 25, 2024
% remark: this code is by no means optimized

function [M, Jb_ee] = GIM_BodyFixed(q)

if isa(q,'sym') 
    symbolic_flag = true;
else
    symbolic_flag = false;
end
    
global Param;
global ee;
global n; % DOF, number of joints

global Mb; % REMOVE !!!

if isfield(Param(1), 'A')
    % Using IFR Screw Coordinates
    % Initialization for the first body
    FK(1).f = SE3Exp(Param(1).Y,q(1));
    FK(1).C = FK(1).f*Param(1).A;
    % Compute FK for each body
    for i = 2:n
        FK(i).f = FK(i-1).f*SE3Exp(Param(i).Y,q(i));
        FK(i).C = FK(i).f*Param(i).A;
    end
elseif isfield(Param(1), 'B')
    % Using Body Fixed Screw Coordinates
    % Initialization for the first body
    FK(1).C = Param(1).B*SE3Exp(Param(1).X,q(1));
    % Compute FK for each body
    for i = 2:n
        FK(i).C = FK(i-1).C*Param(i).B*SE3Exp(Param(i).X,q(i));
    end
else
    disp('Absolute (A) or Relative (B) configuration of the bodies should be provided in Param structure!');
    return;
end

fkin= FK(n).C*ee;

if symbolic_flag
    disp('FKin computed! Symbolic simplification starts now.');
    fkin = simplify(fkin);
    disp('FKin simplified! Code generation starts now.');
    matlabFunction(fkin,'file','generated_code/Fwd_Kin');
    ccode(fkin,'file','generated_code/c_code/Fwd_Kin.c');
end

% Block diagonal matrix A (6n x 6n) of the Adjoint of body frame
A = []; 
for i=1:n
    if symbolic_flag
        A = sym(blkdiag(A,eye(6,6)));
    else
        A = blkdiag(A,eye(6,6));
    end
    for j=1:i-1
        Crel = SE3Inv(FK(i).C)*FK(j).C;
        AdCrel = SE3AdjMatrix(Crel); 
        r = 6*(i-1)+1;
        c = 6*(j-1)+1;
        A(r:r+5,c:c+5) = AdCrel;       
    end
end

% Block diagonal matrix X (6n x n) of the screw coordinate vector associated to all joints in the body frame (Constant)
X = []; 
for i=1:n
X = blkdiag(X,Param(i).X);
end

% System level Jacobian 
J = A*X;

% Computing the body jacobian of EE
Jb=J(end-5:end,:); %% Body fixed Jacobian of last moving body (This may not correspond to end-effector frame)
Jb_ee = SE3AdjMatrix(SE3Inv(ee))*Jb;  %Body fixed Jacobian of end-effector frame

% Block Diagonal Mb (6n x 6n) Mass inertia matrix in body frame (Constant)
Mb = []; 
for i=1:n
Mb = blkdiag(Mb,Param(i).Mb);
end

% Mass inertia matrix in joint space (n x n)
M = J'*Mb*J;


if symbolic_flag
    disp('GIM computed! Symbolic simplification starts now.');
    M = simplify(M);
    disp('GIM: Symbolic simplification completed!');
    disp('Code generation starts now.');
    matlabFunction(M,'file','GIM_M');
    disp('Code generation finished.');
end

