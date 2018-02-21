function plotRobot(footPose, lengths, angles)
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

%% "unpack" link lengths from "lengths" input matrix:
theta1 = angles(1,1);
theta2 = angles(1,2);
% theta3 = angles(1,3);

phi1 = angles(2,1);
phi2 = angles(2,2);
% phi3 = angles(2,3);

psi1 = angles(3,1);
psi2 = angles(3,2);
% psi3 = angles(3,3);

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

%% FK for theta-chain:
% Calculate (x,y) location of "hip" joint:
xHtheta = -B1x;
yHtheta = B1y;

% Calculate (x,y) location of "knee" joint:
xKtheta = xHtheta + L1*cos(theta1);
yKtheta = yHtheta + L1*sin(theta1);

% Calculate (x,y,angle) of "lower ankle" joint, using FK:
xAtheta = xKtheta + L2*cos(theta1 + theta2);
yAtheta = yKtheta + L2*sin(theta1 + theta2);

%% FK for phi-chain:
% Calculate (x,y) location of "hip" joint:
xHphi = 0;
yHphi = 0;

% Calculate (x,y) location of "knee" joint:
xKphi = xHphi + L3*cos(phi1);
yKphi = yHphi + L3*sin(phi1);

% Calculate (x,y,angle) of "upper ankle" joint, using FK:
xAuphi = xKphi + L4*cos(phi1 + phi2);
yAuphi = yKphi + L4*sin(phi1 + phi2);

%% IK for psi-chain:
% Calculate (x,y) location of "hip" joint:
xHpsi = B2x;
yHpsi = B2y;

% Calculate (x,y) location of "knee" joint:
xKpsi = xHpsi + L5*cos(psi1);
yKpsi = yHpsi + L5*sin(psi1);

% Calculate (x,y,angle) of "lower ankle" joint, using FK:
xApsi = xKpsi + L6*cos(psi1 + psi2);
yApsi = yKpsi + L6*sin(psi1 + psi2);

%% Plot
% figure % plot in a new window
hold off
plot(0,0,'ro','MarkerSize',14); % mark body frame location
hold on
% axis([-10 10 -35 5])
axis([-0.3 0.3 -0.5 0.1])
% axis([-3 3 -4.5 1.5]); % set figure size

% theta-chain:
line([0,xHtheta],[0,yHtheta],'Color','k'); % draw link B1
plot(xHtheta,yHtheta,'go','MarkerSize',10); % mark hip location (theta-chain)
line([xHtheta,xKtheta],[yHtheta,yKtheta],'Color','k'); % draw link 1
plot(xKtheta,yKtheta,'go','MarkerSize',10); % mark knee location (theta-chain)
line([xKtheta,xAtheta],[yKtheta,yAtheta],'Color','k'); % draw link 2
plot(xAtheta,yAtheta,'go','MarkerSize',14); % mark lower ankle location (calculated from FK)

% phi-chain:
plot(xAuphi,yAuphi,'go','MarkerSize',14); % mark upper ankle location (calculated from FK)
line([xHphi,xKphi],[yHphi,yKphi],'Color','k'); % draw link 3
plot(xKphi,yKphi,'go','MarkerSize',10); % mark knee location (phi-chain)
line([xKphi,xAuphi],[yKphi,yAuphi],'Color','k'); % draw link 4

% psi-chain:
line([0,xHpsi],[0,yHpsi],'Color','k'); % draw link B2
plot(xHpsi,yHpsi,'go','MarkerSize',10); % mark hip location (psi-chain)
line([xHpsi,xKpsi],[yHpsi,yKpsi],'Color','k'); % draw link 5
plot(xKpsi,yKpsi,'go','MarkerSize',10); % mark knee location (psi-chain)
line([xKpsi,xApsi],[yKpsi,yApsi],'Color','k'); % draw link 2
plot(xApsi,yApsi,'go','MarkerSize',18); % mark lower ankle location (calculated from FK)

% most distal leg (configuration determined by foot pose):
plot(xAu,yAu,'go','MarkerSize',10); % mark upper ankle location (calculated from foot pose)
line([xAu,xA],[yAu,yA],'Color','k'); % draw link 7
plot(xA,yA,'go','MarkerSize',10); % mark lower ankle location (calculated from foot pose)
line([xA,xF],[yA,yF],'Color','k'); % draw link 8
plot(xF,yF,'bo','MarkerSize',12); % mark foot location

mytitleText1 = ['x = ',num2str(xF),', y = ',num2str(yF),' , \angle = ',...
    num2str(angF*(180/pi)),'\circ'];
mytitleText2 = ['\theta_1 = ',num2str(theta1*(180/pi)),'\circ, \phi_1 = ',...
    num2str(phi1*(180/pi)),'\circ, \psi_1 = ',num2str(psi1*(180/pi)),'\circ'];
title({mytitleText1,mytitleText2},'Interpreter','tex' ); 

axis square;
end % end function