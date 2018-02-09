function qu = NRqu(qa,qu0,lengths,thresh)
% Description:
%   Computes the foot pose, using the actuated joint positions and the
%   constraint Jacobian. Uses a Newton-Raphson root finder to solve for the
%   unactuated joint positions, then solves the forward kinematics problem
%   along one of the three open sub-chains.
%
% Inputs:
%   qa:     actuated joint positions (column vector)
%       qa = [theta1; phi1; psi1];
%   qu0:    initial guess at unactuated joint positions (column vector)
%       qu = [theta2; theta3; phi2; phi3; psi2; psi3];
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1; B2], where
%       B1, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2, L5, L6, and L8 correspond to the psi-chain.
%   thresh: convergence threshold
%
% Outputs:
%   qu: approximate angles of unactuated joints, in a 1x6 row vector
%       qu = [theta2; theta3; phi2; phi3; psi2; psi3];
%
% Dependencies:
%   constraintVector.m
%   constraintJacobian.m

convMeasure = thresh*10; % initialize convergence measure

while(convMeasure > thresh)
    G0 = constraintVector(qa,qu0,lengths);
    Jc = constraintJacobian(qa,qu0,lengths);
    qu1 = qu0 - pinv(Jc)*G0;
    G1 = constraintVector(qa,qu1,lengths);
    convMeasure = norm(G0 - G1)/norm(G0); % compute convergence measure with candidate soln.
    qu0 = qu1; % update approximate unactuated joint positions
end % end Newton-Raphson root finding algorithm

qu = qu0;
end