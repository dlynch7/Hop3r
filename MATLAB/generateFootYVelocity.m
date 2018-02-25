% Calculate actuated joint angle velocity curves for testing of kinematic
% configuration specification in TREP. Robot joint angle and link length
% conventions are found in Figure 2 of planar3DOF.pdf.
close all; clc;

dt = 0.01;
T = 10;
lengths = [15 15 10 10 15 15 5 15 10 10 -10 -10]'; % in centimeters
t = 0:dt:T;

% specify desired y-coord trajectory of foot
Yfoot = -38 + 1.5*sin(t); 

actuatedAngs = zeros(3,length(t));
for i = 1:length(t)
    %set desired foot pose for particular time step
    %x-coord is 0, y deterimined by Yfoot vector, and theta is -pi/2
    %(straight vertical)
    footpose = [0 Yfoot(i) -pi/2]';
    
    %compute all joint angles using subchainIK function
    angles = subchainIK(footpose, lengths, 0);
    pause(0.0001)
    %extract the values of the actuated joints 
    actuatedAngs(:,i) = angles(:,1);
end
actuatedAngs = actuatedAngs';
%compute actuated joint velocites
actuatedVels = diff(actuatedAngs) / dt;

%export to CSV file - to be imported into python/trep
csvwrite('JointAngles.csv',actuatedAngs(1:end-1,:))
csvwrite('JointVelocities.csv',actuatedVels);

initialConds = subchainIK([0 Yfoot(1) -pi/2], lengths, 1)

%plot results
figure
plot(t,Yfoot)
title('Y position of foot vs. time')
figure
plot(t,actuatedAngs)
title('Actuated Joint Angles')
legend('Left','Middle','Right')
figure
plot(t(1:end-1),actuatedVels)
title('Actuated Joint Velocities')
legend('Left','Middle','Right')
