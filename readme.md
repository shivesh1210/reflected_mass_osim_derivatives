# Introduction
This repository contains the MATLAB code for computing reflected mass, operational space inertia matrix and their derivatives. This repository accompanies the IEEE-TRO 2024 submission: Andreas Mueller, Shivesh Kumar, Lie Group O(n) Algorithm for the Reflected Mass, Operational Space Inertia, and Analytic Derivatives
with Application to Optimal Inverse Kinematics.

# Algorithms
Implementation of the different algorithms for calculating reflected mass, operational space inertia matrix and their derivatives are provided in the algorithms folder. Make sure it is added to the MATLAB path. 

* Recursive_Diff_DRM_InvOSIM.m => Recursive computation of partial derivative of the DRM implicitly based on the OSIM (Algorithm 2)

* Recursive_Diff_InvOSIM.m => Recursive computation of partial derivative of inverse OSIM (Algorithm 6)

* Recursive_Diff_ReflectedMass.m => Recursive computation of partial derivatives of the DRM based on
solution of the gen. momentum equation using articulated body inertia. The robot Jacobian is not needed. (Algorithm 4 with forward recursion 1b).

* Recursive_Diff_ReflectedMass_Jac.m => Recursive computation of partial derivatives of the DRM based on solution of the gen. momentum equation using articulated body inertia. The robot Jacobian is provided. (Algorithm 4 with forward recursion 1a)

* Recursive_DRM_InvOSIM.m => Recursive computation of the DRM implicitly based on the OSIM (Algorithm 1)

* Recursive_Grad_DRM.m => Recursive computation of gradient of the DRM based on solution of the gen. momentum equation using articulated body inertia. The robot Jacobian is not needed. (Algorithm 4 with forward recursion 1b)

* Recursive_Grad_DRM_InvOSIM.m => Recursive computation of gradient of the DRM implicitly based on the OSIM (Algorithm 2)

* Recursive_Grad_DRM_Jac.m => Recursive computation of gradient of the DRM based on solution of the gen. momentum equation using articulated body inertia. The robot Jacobian is provided. (Algorithm 4 with forward recursion 1a)

* Recursive_InvOSIM.m => Recursive computation of the inverse OSIM (Algorithm 5)

* Recursive_ReflectedMass.m => Recursive computation of the DRM based on solution of the gen. momentum equation using articulated body inertia. The robot Jacobian is not needed. (Algorithm 3 with forward recursion 1b)

* Recursive_ReflectedMass_Jac.m => Recursive computation of the DRM based on solution of the gen. momentum equation using articulated body inertia. The robot Jacobian is provided. (Algorithm 3 with forward recursion 1a)

* GIM_BodyFixed.m => Calculate the Generalized Inertia Matrix in Closed form

# Helper functions
Basic functions from Lie group theory for the relevant matrix Lie group SE(3) are provided in the helper_functions folder. Make sure it is added to the MATLAB path. 

* SE3Exp.m: Function to compute exponential mapping for SE(3)
* SO3Exp.m: Function to compute exponential mapping for SO(3)
* SE3Inv.m: Function to compute analytical inverse of exponential mapping for SE(3)
* SE3AdjMatrix.m: Function to compute (6x6) Adjoint Matrix for SE(3)
* SE3adMatrix.m: Function to compute (6x6) adjoint Matrix for SE(3) - also known as spatial cross product in the literature
* SE3AdjInvMatrix.m: Function to compute Inverse of (6x6) Adjoint Matrix for SE(3)
* se3ToR6.m: Function to map an element from se(3) to R^6
* InertiaMatrix.m => Builds 3x3 inertia matrix from minimal set of parameters
* MassMatrixMixedData.m => Builds the 6x6 mass-inertia matrix of a body with mass, COM, and inertia matrix

# Iterative solution of Optimal DRM minimizing Inverse Kinematics
* IK_ReflectedMass.m => Calculate the optimal inverse kinematics by minimizing the directed reflected mass (DRM)
* ForwardKinRecursive.m => Calculate the forward kinematics i.e. homogenous transformation matrix of end effector frame and its associated Jacobian matrix used in IK_ReflectedMass method.

# Sets the robot parameters
Panda_SetParam.m => Initialize the robot with parameters of the Panda Emika robot. Modify this file to change the robot model. 

# Example 1: Pick-place trajectory
Panda_Traj1_Compute_q0.m => Compute IK solution of the Panda robot for the pick-place motion
Panda_Traj1_Compute.m => Compute trajectory of the Panda robot for the pick-place motion
Panda_Traj1_Plot.m => Plot the results for the Panda robot performing the pick-place motion
Panda_Traj1_Animate.m => 3D-Animation of the Panda robot performing the pick-place motion

# Example 2: Circular trajectory
Panda_Traj2_Compute_q0.m => Compute IK solution of the Panda robot for the circular motion
Panda_Traj2_Compute.m => Compute trajectory of the Panda robot for the circular motion
Panda_Traj2_Plot.m => Plot the results for the Panda robot performing the circular motion
Panda_Traj2_Animate.m => 3D-Animation of the Panda robot performing the circular motion

# Example 3: Compute two IK solutions
Panda_IK_DRM_min.m => Calculate a DRM-minimizing solution starting from an inital pose with q0 = [pi/4,0,-pi/4,-pi/2,0,pi/2,0]

# Results
Results generated from Examples 1 and 2 are stored in the results folder. 
* q_Traj1_Grad.txt
* q_Traj2_Grad.txt
* q_Traj1_PI.txt
* q_Traj2_PI.txt
* RefMass_Traj1_Grad.txt
* RefMass_Traj1_PI.txt
* RefMass_Traj2_Grad.txt
* RefMass_Traj2_PI.txt
* Error_Traj2_Grad.txt
* Error_Traj1_Grad.txt
* Error_Traj1_PI.txt
* Error_Traj2_PI.txt
* nIter_Traj1_Grad.txt
* nIter_Traj1_PI.txt
* nIter_Traj2_Grad.txt
* nIter_Traj2_PI.txt



