function footPose = NRFK(qa,qu0,lengths,thresh,chain)
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
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x; B1y; B2y], where
%       B1x, B1y, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2x, B2y, L5, L6, and L8 correspond to the psi-chain.
%   thresh: convergence threshold
%   chain:  select which open sub-chain to use for FK
%       1: use the theta-chain
%       2: use the phi-chain
%       3: use the psi-chain
%
% Outputs:
%   footPose:   3-element row vector which comprises:
%       xF:     x-coordinate of the foot, relative to the body
%       yF:     y-coordinate of the foot, relative to the body
%       angF:   angle of the foot, relative to the body.
%
% Dependencies:
%   constraintVector.m
%   constraintJacobian.m
%   subchainFK.m

convMeasure = thresh*10; % initialize convergence measure

G0 = zeros(6,1);
G1 = zeros(6,1);

convMeasurePrev = convMeasure*100; % initialize previous convergence measure

while(convMeasure > thresh)
    [G0] = constraintVector(qa,qu0,lengths);
    Jc = constraintJacobian(qa,qu0,lengths);
    qu1 = qu0 - pinv(Jc)*G0; % candidate step
    [G1] = constraintVector(qa,qu1,lengths);
    convMeasure = norm(G0 - G1)/norm(G0); % compute convergence measure with candidate soln.
    fprintf('convergence measure: %f\n',convMeasure);
    % perform a binary search:
    if convMeasure < convMeasurePrev %% if the approximation is converging
        qu0 = qu1; % update approximate unactuated joint positions
        convMeasurePrev = convMeasure; % prepare for next iteration
        fprintf('took full step\n');
    else
        qu0 = (qu0 + qu1)./2; % take a smaller step and try again
        fprintf('halved step size\n');
    end
end % end Newton-Raphson root finding algorithm

theta = [qa(1); qu0(1); qu0(2)]; % theta-chain joint positions
phi = [qa(2); qu0(3); qu0(4)]; % phi-chain joint positions
psi = [qa(3); qu0(5); qu0(6)]; % psi-chain joint positions

angles = transpose([theta(:), phi(:), psi(:)]);

switch chain
    case 1 % calculate FK along theta-chain
        [xF, yF, angF] = subchainFK(angles,lengths,1);
    case 2 % calculate FK along phi-chain
        [xF, yF, angF] = subchainFK(angles,lengths,2);
    case 3 % calculate FK along psi-chain
        [xF, yF, angF] = subchainFK(angles,lengths,3);
    otherwise % handle errors
        disp('NRFK error: unknown open sub-chain selection');
end

footPose = [xF, yF, angF];

end