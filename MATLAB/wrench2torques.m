function [torques] = wrench2torques(angles,lengths,wrench)
%% Description:
% uses the actuator Jacobian to map a specified end-effector wrench to joint torques
% Inputs:
%   angles: 3x3 array of joint angles in radians
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x], where
%       B1x, B1y, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2x, B2y, L5, L6, and L8 correspond to the psi-chain.
%   wrench: end-effector wrench (3x1 column vector):
%       wrench = [Fx; Fy; Tz];
% Outputs:
%   [torques]: joint torques (3x1 column vector)
%       [torques] = [T1; T2; T3];
% Dependencies:
%   actuatorJacobian(...)
%

qa = [angles(1,1); angles(2,1); angles(3,1)];
qu = [angles(1,2); angles(1,3); angles(2,2); angles(2,3); angles(3,2); angles(3,3)];
Ja = actuatorJacobian(qa, qu, lengths, 1);
torques = transpose(Ja)*wrench;
end