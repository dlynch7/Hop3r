function [xF, yF, angF] = subchainFK(angles, lengths, chain)
%% Description:
%   Computes forward kinematics along a sub-chain (theta, phi, and psi
%   chains)
%
% Inputs:
%   angles: a 3x3 matrix of the angles along the open sub-chain
%       specified by "chain" (theta, phi, or psi). Organized thusly:
%       angles = [th1,  th2,    th3;
%                 ph1,  ph2,    ph3;
%                 ps1,  psi2,   psi3];
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x; B1y; B2y], where
%       B1x, B1y, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2x, B2y, L5, L6, and L8 correspond to the psi-chain.
%   chain: a numeric option through which the user specifies which open
%       sub-chain shall be used to compute the forward kinematics.
%       1: theta-chain
%       2: phi-chain
%       3: psi-chain
%
% Outputs:
%   xF:     x-coordinate of the foot, relative to the body
%   yF:     y-coordinate of the foot, relative to the body
%   angF:   angle of the foot, relative to the body.
%
% Dependencies:
%   none

%% "unpack" link lengths from "lengths" input vector:
% Note: this step is just for cleaner-looking code.
L1 = lengths(1);
L2 = lengths(2);
L3 = lengths(3);
L4 = lengths(4);
L5 = lengths(5);
L6 = lengths(6);
L7 = lengths(7);
L8 = lengths(8);
B1x = lengths(9);
B2x = lengths(10);
B1y = lengths(11);
B2y = lengths(12);

%% "unpack joint angles from "angles" input matrix:
% Note: this step is just for cleaner-looking code.
% theta-chain:
th1 = angles(1,1);
th2 = angles(1,2);
th3 = angles(1,3);

% phi-chain:
ph1 = angles(2,1);
ph2 = angles(2,2);
ph3 = angles(2,3);

% psi-chain:
ps1 = angles(3,1);
ps2 = angles(3,2);
ps3 = angles(3,3);

%% Compute FK according to "chain" input option
switch chain
    case 1 % theta-chain
        xF = -B1x + L1*cos(th1) + L2*cos(th1+th2) + L8*cos(th1+th2+th3);
        yF = B1y + L1*sin(th1) + L2*sin(th1+th2) + L8*sin(th1+th2+th3);
        angF = th1+th2+th3;
        
    case 2 % phi-chain
        xF = L3*cos(ph1) + L4*cos(ph1+ph2) + (L7+L8)*cos(ph1+ph2+ph3);
        yF = L3*sin(ph1) + L4*sin(ph1+ph2) + (L7+L8)*sin(ph1+ph2+ph3);
        angF = ph1+ph2+ph3;
        
    case 3 % psi-chain
        xF = B2x + L5*cos(ps1) + L6*cos(ps1+ps2) + L8*cos(ps1+ps2+ps3);
        yF = B2y + L5*sin(ps1) + L6*sin(ps1+ps2) + L8*sin(ps1+ps2+ps3);
        angF = ps1+ps2+ps3;
    otherwise % error handling
        disp('subchainFK error: unknown unknown open sub-chain selection');
end

end