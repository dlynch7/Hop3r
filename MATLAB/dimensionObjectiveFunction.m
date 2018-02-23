function J = dimensionObjectiveFunction(lengths, angles, wrench, twist, torqueWeight, velWeight)
%% Description:
% Computes a quadratic objective function (for optimization) using torques
% and joint velocities, calculated from an input wrench and an input twist
% using the actuator Jacobian (which is a function of link lengths and
% joint angles).

%% Inputs:
% lengths: 12x1 array
%   [L1; L2; L3; L4; L5; L6; L7; L8; B1x; B1y; B2x; B2y];

% angles: 3x3 array
%   [theta1 theta2  theta3;
%    phi1   phi2    phi3;
%    psi1   psi2    psi3];

% wrench: 3x1 array
%   [Fx; Fy; Mz];

% twist: 3x1 array
%   [vx; vy; wz];

% torqueWeight: 3x3 positive-definite array. Simplest case is a multiple of
% eye(3);
%   [tW1a tW1b tW1c;
%    tW2a tW2b tW2c;
%    tW3a tW3b tW3c];

% velWeight: 3x3 positive-definite array. Simplest case is a multiple of
% eye(3);
%   [vW1a vW1b vW1c;
%    vW2a vW2b vW2c;
%    vW3a vW3b vW3c];

%% Calculate joint torques (Nm) from input wrench:
torques = wrench2torques(angles, lengths, wrench);

%% Calculate joint velocities (rad/s) from input twist:
jointVels = twist2vels(angles, lengths, twist);

%% Calculate value of objective function:
J = (transpose(torques)*torqueWeight*torques) + ...
    (transpose(jointVels)*velWeight*jointVels);
end