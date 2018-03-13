function Ja = actuatorJacobian(qa,qu,lengths,chain)
%% Description:
% Calculates the actuator Jacobian
%
% Inputs:
%   qa: actuated joint positions (3x1 column vector) 
%       qa = [theta1; phi1; psi1];
%   qu: unactuated joint positions (6x1 column vector)
%       qu = [theta2; theta3; phi2; phi3; psi2; psi3];
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1; B2], where
%       B1, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2, L5, L6, and L8 correspond to the psi-chain.
%   chain: the actuator Jacobian will be calculated using this open
%   subchain's Jacobian.
%       chain = 1: use Jtheta (theta-chain Jacobian)
%       chain = 2: use Jphi (phi-chain Jacobian)
%       chain = 3: use Jpsi (psi-chain Jacobian
%
% Outputs:
%   Ja: the actuator Jacobian, a 3x3 matrix.
%
% Dependencies:
%   constraintJacobian.m
%   subchainJacobian.m
%
%% Get the constraint Jacobian
Jc = constraintJacobian(qa,qu,lengths);

%% Get the all open subchain Jacobians
Jtheta = subchainJacobian(qa,qu,lengths,1);
Jphi = subchainJacobian(qa,qu,lengths,2);
Jpsi = subchainJacobian(qa,qu,lengths,3);

%% Form Ha from open subchain Jacobians
Ha = [Jtheta(1,1), -Jphi(1,1), 0;
      Jtheta(2,1), -Jphi(2,1), 0;
      Jtheta(3,1), -Jphi(3,1), 0;
      0, -Jphi(1,1), Jpsi(1,1);
      0, -Jphi(2,1), Jpsi(2,1);
      0, -Jphi(3,1), Jpsi(3,1)];
  
%% Return the specified actuator Jacobian
switch chain
    case 1
        Ja = Jtheta*[1 0 0;
            [1 0 0 0 0 0]*(-inv(Jc))*Ha;
            [0 1 0 0 0 0]*(-inv(Jc))*Ha];
    case 2
        Ja = Jphi*[0 1 0;
            [0 0 1 0 0 0]*(-inv(Jc))*Ha;
            [0 0 0 1 0 0]*(-inv(Jc))*Ha];
    case 3
        Ja = Jpsi*[0 0 1;
            [0 0 0 0 1 0]*(-inv(Jc))*Ha;
            [0 0 0 0 0 1]*(-inv(Jc))*Ha];
    otherwise
        error('actuatorJacobian error: unknown open subchain selection');
end

end