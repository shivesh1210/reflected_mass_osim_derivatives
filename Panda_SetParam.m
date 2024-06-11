
global n; % DOF, number of joints
n = 7;

global g_vector; % gravity vector
g_vector = [0; 0; -9.80665];

%% End-effector configuration wrt last link body fixed frame in the chain
re = [0;0;0];
ee = [eye(3),re;[0,0,0],[1]];

%% Joint screw coordinates in spatial representation
Param(1).Y = [0, 0, 1, 0., 0., 0]';
Param(2).Y = [0, 1, 0, -0.333, 0., 0]';
Param(3).Y = [0, 0, 1, 0., 0., 0]';
Param(4).Y = [0, -1, 0, 0.649, 0., -0.0825]';
Param(5).Y = [0, 0, 1, 0, 0., 0]';
Param(6).Y = [0, -1, 0, 1.033, 0., 0]';
Param(7).Y = [0, 0, -1, 0., 0.088, 0]';

%% Reference configurations of bodies (i.e. of bdoy-fixed reference frames)
Param(1).A = [eye(3),[0,0,0.333]';[0,0,0],[1]];
Param(2).A = [SO3Exp([1,0,0],-pi/2),[0,0,0.333]';[0,0,0],[1]];
Param(3).A = [eye(3),[0,0,0.649]';[0,0,0],[1]];
Param(4).A = [SO3Exp([1,0,0],pi/2),[0.0825,0,0.649]';[0,0,0],[1]];
Param(5).A = [eye(3),[0,0,1.033]';[0,0,0],[1]];
Param(6).A = [SO3Exp([1,0,0],pi/2),[0.0,0,1.033]';[0,0,0],[1]];
Param(7).A = [SO3Exp([1,0,0],pi),[0.088,0,1.033]';[0,0,0],[1]];

Param(1).B = Param(1).A;
Param(2).B = SE3Inv(Param(1).A)*Param(2).A;
Param(3).B = SE3Inv(Param(2).A)*Param(3).A;
Param(4).B = SE3Inv(Param(3).A)*Param(4).A;
Param(5).B = SE3Inv(Param(4).A)*Param(5).A;
Param(6).B = SE3Inv(Param(5).A)*Param(6).A;
Param(7).B = SE3Inv(Param(6).A)*Param(7).A;

%% Joint screw coordinates in body-fixed representation
Param(1).X = SE3AdjInvMatrix(Param(1).A)*Param(1).Y;
Param(2).X = SE3AdjInvMatrix(Param(2).A)*Param(2).Y;
Param(3).X = SE3AdjInvMatrix(Param(3).A)*Param(3).Y;
Param(4).X = SE3AdjInvMatrix(Param(4).A)*Param(4).Y;
Param(5).X = SE3AdjInvMatrix(Param(5).A)*Param(5).Y;
Param(6).X = SE3AdjInvMatrix(Param(6).A)*Param(6).Y;
Param(7).X = SE3AdjInvMatrix(Param(7).A)*Param(7).Y;

Param(1).Xbar = SE3AdjMatrix(Param(1).B)*Param(1).X;
Param(2).Xbar = SE3AdjMatrix(Param(2).B)*Param(2).X;
Param(3).Xbar = SE3AdjMatrix(Param(3).B)*Param(3).X;
Param(4).Xbar = SE3AdjMatrix(Param(4).B)*Param(4).X;
Param(5).Xbar = SE3AdjMatrix(Param(5).B)*Param(5).X;
Param(6).Xbar = SE3AdjMatrix(Param(6).B)*Param(6).X;
Param(7).Xbar = SE3AdjMatrix(Param(7).B)*Param(7).X;

%% Intertia paramater as reported in [C. Gaz, 2019]
Param(1).Mb = MassMatrixMixedData(4.970684, ...
    [0.70337,-1.39e-04,6.772e-03;
    -1.39e-04,0.70661,1.9169e-02;
    6.772e-03,1.9169e-02,9.117e-03], ...
    [3.875e-03, 2.081e-03, -0.1750]);
Param(2).Mb = MassMatrixMixedData(0.646926, ...
    [7.962e-03, -3.925e-03, 1.0254e-02;
     -3.925e-03, 2.811e-02, 7.04e-04;
     1.0254e-02, 7.04e-04, 2.5995e-02], ...
    [-3.141e-03, -2.872e-02, 3.495e-03]);
Param(3).Mb = MassMatrixMixedData(3.228604, ...
    [3.7242e-02, -4.761e-03, -1.1396e-02;
     -4.761e-03, 3.6155e-02, -1.2805e-02;
     -1.1396e-02, -1.2805e-02, 1.083e-02], ...
    [2.7518e-02, 3.9252e-02, -6.6502e-02]);
Param(4).Mb = MassMatrixMixedData(3.587895, ...
    [2.5853e-02, 7.796e-03, -1.332e-03;
     7.796e-03, 1.9552e-02, 8.641e-03;
     -1.332e-03, 8.641e-03, 2.8323e-02], ...
    [-5.317e-02, 0.104419, 2.7454e-02]);
Param(5).Mb = MassMatrixMixedData(1.225946, ...
    [3.5549e-02, -2.117e-03, -4.037e-03;
     -2.117e-03, 2.9474e-02, 2.29e-04;
     -4.037e-03, 2.29e-04, 8.627e-03], ...
    [-1.1953e-02, 4.1065e-02, -3.8437e-02]);
Param(6).Mb = MassMatrixMixedData(1.666555, ...
    [1.964e-03, 1.09e-04, -1.158e-03;
     1.09e-04, 4.354e-03, 3.41e-04;
     -1.158e-03, 3.41e-04, 5.433e-03], ...
    [6.0149e-02, -1.4117e-02, -1.0517e-02]);
Param(7).Mb = MassMatrixMixedData(0.735522, ...
    [1.2516e-02, -4.28e-04, -1.196e-03;
     -4.28e-04, 1.0027e-02, -7.41e-04;
     -1.196e-03, 7.41e-04, 4.815e-03], ...
    [1.0517e-02, -4.252e-03, 6.1597e-02]);
