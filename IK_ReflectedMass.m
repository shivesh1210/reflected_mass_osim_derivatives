
function [qd,nIter] = IK_ReflectedMass(q,Cd,h,alpha,nIterMax,Accuracy,minGradNorm)

global Param;
global Chain;
n = length(Param);

qd = q;
i=1;
[Jee,Cee] = ForwardKinRecursive(qd);
DeltaC = SE3Inv(Cee)*Cd;
Xerr = se3ToR6(DeltaC-eye(4));
% grad = GradReflectedMass(qd,h);
% grad = Recursive_Grad_DRM(qd,h);
% grad = Recursive_Grad_DRM_Jac(qd,h,Jee);
grad = Recursive_Grad_DRM_InvOSIM(qd,h);
[Jee,Cee] = ForwardKinRecursive(qd);
Jinv = pinv(Jee);
N = eye(n) - Jinv*Jee;

while (i<=nIterMax) & (norm(Xerr) > Accuracy) || (alpha*norm(N*grad') > minGradNorm)
    [Jee,Cee] = ForwardKinRecursive(qd);
    DeltaC = SE3Inv(Cee)*Cd;
    Xerr = se3ToR6(DeltaC-eye(4));
    Jinv = pinv(Jee);
    N = eye(n) - Jinv*Jee;
%     grad = GradReflectedMass(qd,h);
%     grad = Recursive_Grad_DRM(qd,h);
%     grad = Recursive_Grad_DRM_Jac(qd,h,Jee);
    grad = Recursive_Grad_DRM_InvOSIM(qd,h);
    deltaq = Jinv*Xerr' - alpha*N*grad';
    qd = qd + deltaq;
    i = i+1;
end
nIter = i;
