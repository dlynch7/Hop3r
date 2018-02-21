%% Calculate torques using actuator Jacobian
L1 = 76.2/1000;
L2 = 120.65/1000;
L3 = 76.2/1000;
L4 = 76.2/1000;
L5 = L1;
L6 = L2;
L7 = 50.8/1000;
L8 = 165.1/1000;
L9 = 101.6/1000;
L10 = 101.6/1000;
L11 = 20/1000;
L12 = 20/1000;

L = [L1; L2; L3; L4; L5; L6; L7; L8; L9; L10; L11; L12];

footPose = [0; -279.4/1000; -pi/2];

[angles] = subchainIK(footPose, L, 1);

qa = [angles(1,1); angles(2,1); angles(3,1)];

qu = [angles(1,2); angles(1,3); angles(2,2); angles(2,3); angles(3,2); angles(3,3)];

Ja = actuatorJacobian(qa, qu, L, 1);

Fx = 0;
Fy = 0;
Mz = 0;

% wrench = [Mz; Fx; Fy];
wrench = [Fx; Fy; Mz];

torques = transpose(Ja)*wrench;