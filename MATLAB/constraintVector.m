function [g] = constraintVector(qa,qu,lengths)
%% Description:
%   Calculates a 6x1 vector of the loop-closure expressions.
%
% Inputs:
%   qa: actuated joint positions (3x1 column vector) 
%       qa = [theta1; phi1; psi1];
%   qu: unactuated joint positions (6x1 column vector)
%       qu = [theta2; theta3; phi2; phi3; psi2; psi3];
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x; B1y; B2y], where
%       B1x, B1y, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2x, B2y, L5, L6, and L8 correspond to the psi-chain.
%
% Outputs:
%   g: the 6x1 column vector containing the loop-closure expressions
%
% Dependencies:
%   subchainFK.m - computes FK along each sub-chain (theta, phi, and psi
%   chains)

%% construct 3x3 matrix of joint angles from "qa" and "qu" input vectors
angles = [qa(1), qu(1), qu(2);
          qa(2), qu(3), qu(4);
          qa(3), qu(5), qu(6)];
      
Tbftheta = zeros(3,1);
Tbfphi = zeros(3,1);
Tbfpsi = zeros(3,1);
      
g = zeros(6,1);

[Tbftheta(1),Tbftheta(2),Tbftheta(3)] = subchainFK(angles,lengths,1);
[Tbfphi(1),Tbfphi(2),Tbfphi(3)] = subchainFK(angles,lengths,2);
[Tbfpsi(1),Tbfpsi(2),Tbfpsi(3)] = subchainFK(angles,lengths,3);

g = [Tbftheta(:) - Tbfphi(:);
     -Tbfphi(:) + Tbfpsi(:)];
end