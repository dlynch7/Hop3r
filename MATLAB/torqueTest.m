%% Calculate actuator Jacobian as function of joint angles
L1 = 76.2/1000;
L2 = 120.65/1000;
L3 = 76.2/1000;
L4 = 76.2/1000;
L5 = L1;
L6 = L2;
L7 = 50.8/1000;
L8 = 100/1000;
L9 = 101.6/1000;
L10 = 101.6/1000;
L11 = 20/1000;
L12 = 20/1000;

L = [L1; L2; L3; L4; L5; L6; L7; L8; L9; L10; L11; L12];

footPose = [0; -150/1000; -pi/2];

[angles] = subchainIK(footPose, L, 1);

qa = [angles(1,1); angles(2,1); angles(3,1)];

qu = [angles(1,2); angles(1,3); angles(2,2); angles(2,3); angles(3,2); angles(3,3)];

Ja = actuatorJacobian(qa, qu, L, 1);

%% map end-effector wrench to joint torques
Fx = 0;
Fy = -71.5;
Mz = 0;

% wrench = [Mz; Fx; Fy];
wrench = [Fx; Fy; Mz];

torques = transpose(Ja)*wrench;

%% map end-effector twist to joint velocities
vx = 0;
vy = -1.4;
wz = 0;

twist = [vx; vy; wz];

jointVels = pinv(Ja)*twist; % joint velocities in rad/s
jointVelsRPM = jointVels.*(60/(2*pi)); % convert to rpm

%% compute value of objective function:
TW = eye(3);
VW = eye(3);
J = dimensionObjectiveFunction(L,angles,wrench,twist,TW,VW);
fprintf("Objective function value: %f\n",J);