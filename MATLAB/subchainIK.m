function [angles] = subchainIK(footPose, lengths, plotOption)
%% Description:
%   Computes inverse kinematics for each sub-chain (theta, phi, and psi
%   chains)
%
% Inputs:
%   footPose: a 3x1 column vector representing the foot pose:
%       footPose = [xF;
%                   yF;
%                   angF];
%   lengths: the relevant dimensions of the robot, stored in a 10x1 column
%       vector according to this order:
%       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1; B2], where
%       B1, L1, L2, and L8 correspond to the theta-chain,
%       L3, L4, L7, and L8 correspond to the phi-chain, and
%       B2, L5, L6, and L8 correspond to the psi-chain.
%
% Outputs:
%   angles = [theta1    theta2  theta3;
%             phi1      phi2    phi3;
%             psi1      psi2    psi3];
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

%% Extract foot pose:
xF = footPose(1);
yF = footPose(2);
angF = footPose(3);

% Calculate (x,y) location of "lower ankle" joint, from foot pose:
xA = xF - L8*cos(angF);
yA = yF - L8*sin(angF);

% Calculate (x,y) location of "upper ankle" joint, from foot pose:
xAu = xF - (L7+L8)*cos(angF);
yAu = yF - (L7+L8)*sin(angF);

%% IK for theta-chain:
% Calculate (x,y) location of "hip" joint:
xHtheta = -B1x;
yHtheta = B1y;

% Calculate hip angle:
xAptheta = -xHtheta + xA;
yAptheta = yHtheta - yA;
gammatheta = wrapToPi(abs(atan2(yAptheta,xAptheta)));
alphatheta = wrapToPi(acos((xAptheta^2 + yAptheta^2 + L1^2 - L2^2)/(2*L1*sqrt(xAptheta^2 + yAptheta^2))));
theta1 = wrapToPi(-gammatheta - alphatheta);

% Calculate (x,y) location of "knee" joint:
xKtheta = xHtheta + L1*cos(theta1);
yKtheta = yHtheta + L1*sin(theta1);

% Calculate knee angle:
betatheta = wrapToPi(acos((L1^2 + L2^2 - xAptheta^2 - yAptheta^2)/(2*L1*L2)));
theta2 = wrapToPi(pi - betatheta);

% Calculate (x,y,angle) of "lower ankle" joint, using FK:
xAtheta = xKtheta + L2*cos(theta1 + theta2);
yAtheta = yKtheta + L2*sin(theta1 + theta2);
theta3 = wrapToPi(angF - theta1 - theta2);
            
%% IK for phi-chain:
% Calculate (x,y) location of "hip" joint:
xHphi = 0;
yHphi = 0;

% Calculate hip angle:
gammaphi = wrapToPi(abs(atan2(yAu,xAu)));
alphaphi = wrapToPi(acos((xAu^2 + yAu^2 + L3^2 - L4^2)/(2*L3*sqrt(xAu^2 + yAu^2))));
phi1 = wrapToPi(-gammaphi - alphaphi);

% Calculate (x,y) location of "knee" joint:
xKphi = xHphi + L3*cos(phi1);
yKphi = yHphi + L3*sin(phi1);

% Calculate knee angle:
betaphi = wrapToPi(acos((L3^2 + L4^2 - xAu^2 - yAu^2)/(2*L3*L4)));
phi2 = wrapToPi(pi - betaphi);

% Calculate (x,y,angle) of "upper ankle" joint, using FK:
xAuphi = xKphi + L4*cos(phi1 + phi2);
yAuphi = yKphi + L4*sin(phi1 + phi2);
phi3 = wrapToPi(angF - phi1 - phi2);
            
%% IK for psi-chain:
% Calculate (x,y) location of "hip" joint:
xHpsi = B2x;
yHpsi = B2y;

% Calculate hip angle:
xAppsi = xHpsi - xA;
yAppsi = yHpsi - yA;
gammapsi = wrapToPi(abs(atan2(yAppsi,xAppsi)));
alphapsi = wrapToPi(acos((xAppsi^2 + yAppsi^2 + L5^2 - L6^2)/(2*L5*sqrt(xAppsi^2 + yAppsi^2))));
psi1 = wrapToPi(pi + gammapsi + alphapsi);

% Calculate (x,y) location of "knee" joint:
xKpsi = xHpsi + L5*cos(psi1);
yKpsi = yHpsi + L5*sin(psi1);

% Calculate knee angle:
betapsi = wrapToPi(acos((L5^2 + L6^2 - xAppsi^2 - yAppsi^2)/(2*L5*L6)));
psi2 = wrapToPi(pi + betapsi);

% Calculate (x,y,angle) of "lower ankle" joint, using FK:
xApsi = xKpsi + L6*cos(psi1 + psi2);
yApsi = yKpsi + L6*sin(psi1 + psi2);
psi3 = wrapToPi(angF - psi1 - psi2);

%% Return joint angles:
angles = [theta1    theta2  theta3;
          phi1      phi2    phi3;
          psi1      psi2    psi3];

%% Plot
if plotOption
%     figure
%     plot(0,0,'ro','MarkerSize',14); % mark body frame location
%     hold on
%     axis([-3 3 -4.5 1.5]); % set figure size
% 
%     % theta-chain:
%     line([0,xHtheta],[0,yHtheta],'Color','k'); % draw link B1
%     plot(xHtheta,yHtheta,'go','MarkerSize',10); % mark hip location (theta-chain)
%     line([xHtheta,xKtheta],[yHtheta,yKtheta],'Color','k'); % draw link 1
%     plot(xKtheta,yKtheta,'go','MarkerSize',10); % mark knee location (theta-chain)
%     line([xKtheta,xAtheta],[yKtheta,yAtheta],'Color','k'); % draw link 2
%     plot(xAtheta,yAtheta,'go','MarkerSize',14); % mark lower ankle location (calculated from FK)
% 
%     % phi-chain:
%     plot(xAuphi,yAuphi,'go','MarkerSize',14); % mark upper ankle location (calculated from FK)
%     line([xHphi,xKphi],[yHphi,yKphi],'Color','k'); % draw link 3
%     plot(xKphi,yKphi,'go','MarkerSize',10); % mark knee location (phi-chain)
%     line([xKphi,xAuphi],[yKphi,yAuphi],'Color','k'); % draw link 4
% 
%     % psi-chain:
%     line([0,xHpsi],[0,yHpsi],'Color','k'); % draw link B2
%     plot(xHpsi,yHpsi,'go','MarkerSize',10); % mark hip location (psi-chain)
%     line([xHpsi,xKpsi],[yHpsi,yKpsi],'Color','k'); % draw link 5
%     plot(xKpsi,yKpsi,'go','MarkerSize',10); % mark knee location (psi-chain)
%     line([xKpsi,xApsi],[yKpsi,yApsi],'Color','k'); % draw link 2
%     plot(xApsi,yApsi,'go','MarkerSize',18); % mark lower ankle location (calculated from FK)
% 
%     % most distal leg (configuration determined by foot pose):
%     plot(xAu,yAu,'go','MarkerSize',10); % mark upper ankle location (calculated from foot pose)
%     line([xAu,xA],[yAu,yA],'Color','k'); % draw link 7
%     plot(xA,yA,'go','MarkerSize',10); % mark lower ankle location (calculated from foot pose)
%     line([xA,xF],[yA,yF],'Color','k'); % draw link 8
%     plot(xF,yF,'bo','MarkerSize',12); % mark foot location

plotRobot(footPose, lengths, angles);


end % end if plotOption

end % end function