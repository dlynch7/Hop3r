function [footPose,qu] = geomFK(qa,lengths,solOption)
% Description:
%   Computes the foot pose, using the actuated joint positions and the
%   constraint Jacobian. Uses geometry (intersecting circles) to solve for
%   the locations of the "ankle" (xA,yA) and the "upper ankle" (xuA, yuA)
%   relative to the base frame, then calculates unactuated joint positions
%   and the foot pose
%
% Inputs:
%   qa:     actuated joint positions (column vector)
%       qa = [theta1; phi1; psi1];
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x; B1y; B2y], where
%       B1x, B1y, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2x, B2y, L5, L6, and L8 correspond to the psi-chain.
%   solOption: 1, 2, 3, or 4
%       1: function uses (xA1,yA1) and (xuA1, yuA1)
%       2: function uses (xA1,yA1) and (xuA2, yuA2)
%       3: function uses (xA2,yA2) and (xuA1, yuA1)
%       4: function uses (xA2,yA2) and (xuA2, yuA2)
%
% Outputs:
%   qu:         6x1 column vector
%       qu = [th2; th3; ph; ph3; ps2; ps3];
%   footPose:   3-element row vector which comprises:
%       xF:     x-coordinate of the foot, relative to the body
%       yF:     y-coordinate of the foot, relative to the body
%       angF:   angle of the foot, relative to the body.
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

%% "unpack" actuated joint angles from "qa" input vector:
% Note: this step is just for cleaner-looking code.
th1 = qa(1);
ph1 = qa(2);
ps1 = qa(3);

%% create empty arrays:
qu = zeros(6,1);
footPose = zeros(3,1);

%% Calculate (xA, yA) using th1 and ps1:
xkth = -B1x + L1*cos(th1);  % x-coordinate of theta-chain "knee"
ykth = B1y + L1*sin(th1);   % y-coordinate of theta-chain "knee"

xkps = B2x + L5*cos(ps1);  % x-coordinate of psi-chain "knee"
ykps = B2y + L5*sin(ps1);   % y-coordinate of psi-chain "knee"

a = sqrt((xkth - xkps)^2 + (ykth - ykps)^2);    % intermediate variable
b = (L2^2 - L6^2 + a^2)/(2*a);                  % intermediate variable
c = sqrt(L2^2 - b^2);                           % intermediate variable

xA1 = (b/a)*(xkps - xkth) + (c/a)*(ykps - ykth) + xkth;   % solution 1/2 xA-coordinate
xA2 = (b/a)*(xkps - xkth) - (c/a)*(ykps - ykth) + xkth;   % solution 3/4 xA-coordinate

yA1 = (b/a)*(ykps - ykth) - (c/a)*(xkps - xkth) + ykth;   % solution 1/2 yA-coordinate
yA2 = (b/a)*(ykps - ykth) + (c/a)*(xkps - xkth) + ykth;   % solution 3/4 yA-coordinate

if (solOption==1 || solOption==2)
    xA = xA1;
    yA = yA1;
else
    xA = xA2;
    yA = yA2;
end
fprintf('(xA, yA) = (%f, %f)\n',[xA, yA]);

%% Calculate (xuA, yuA) using ph1 and (xA, yA):
xkph = L3*cos(ph1);  % x-coordinate of phi-chain "knee"
ykph = L3*sin(ph1);   % y-coordinate of phi-chain "knee"

d = sqrt((xkph - xA)^2 + (ykph - yA)^2);    % intermediate variable
e = (L4^2 - L7^2 + d^2)/(2*d);              % intermediate variable
f = sqrt(L4^2 - e^2);                       % intermediate variable

xuA1 = (e/d)*(xA - xkph) - (f/d)*(yA - ykph) + xkph;   % solution 1/3 xuA-coordinate
xuA2 = (e/d)*(xA - xkph) + (f/d)*(yA - ykph) + xkph;   % solution 2/4 xuA-coordinate

yuA1 = (e/d)*(yA - ykph) + (f/d)*(xA - xkph) + ykph;   % solution 1/3 xuA-coordinate
yuA2 = (e/d)*(yA - ykph) - (f/d)*(xA - xkph) + ykph;   % solution 2/4 xuA-coordinate

if (solOption==1 || solOption==3)
    xuA = xuA1;
    yuA = yuA1;
else
    xuA = xuA2;
    yuA = yuA2;
end
fprintf('(xuA, yuA) = (%f, %f)\n',[xuA, yuA]);

%% Solve for "knee" angles (th2, ph2, ps2):

% theta-chain--------------------------------------------------------------
% Calculate (x,y) location of theta-chain "hip" joint:
xHtheta = -B1x;
yHtheta = B1y;

% ankle relative to theta-hip:
xAptheta = -xHtheta + xA;
yAptheta = yHtheta - yA;

% Calculate knee angle:
betatheta = wrapToPi(acos((L1^2 + L2^2 - xAptheta^2 - yAptheta^2)/(2*L1*L2)));
th2 = wrapToPi(pi - betatheta);

fprintf('theta2 = %f degrees\n',th2*(180/pi));

% save th2 to qu array:
qu(1) = th2;

% phi-chain----------------------------------------------------------------
% Calculate knee angle:
betaphi = wrapToPi(acos((L3^2 + L4^2 - xuA^2 - yuA^2)/(2*L3*L4)));
ph2 = wrapToPi(pi - betaphi);

fprintf('phi2 = %f degrees\n',ph2*(180/pi));

% save ph2 to qu array:
qu(3) = ph2;

% psi-chain----------------------------------------------------------------
% Calculate (x,y) location of "hip" joint:
xHpsi = B2x;
yHpsi = B2y;

% Calculate hip angle:
xAppsi = xHpsi - xA;
yAppsi = yHpsi - yA;

% Calculate knee angle:
betapsi = wrapToPi(acos((L5^2 + L6^2 - xAppsi^2 - yAppsi^2)/(2*L5*L6)));
ps2 = wrapToPi(pi + betapsi);

fprintf('psi2 = %f degrees\n',ps2*(180/pi));

% save ps2 to qu array:
qu(5) = ps2;

end