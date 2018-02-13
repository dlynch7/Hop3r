% Calculate actuated joint angle velocity curves for testing of kinematic
% configuration specification in TREP. Robot joint angle and link length
% conventions are found in Figure 2 of planar3DOF.pdf.
close all; clc;

dt = 0.01;
T = 10;
lengths = [10 10 7.5 7.5 10 10 5 15 10 10]'; % in centimeters
t = 0:dt:T;

% specify desired y-coord trajectory of foot
Yfoot = -30 + 2*sin(t); 

actuatedAngs = zeros(3,length(t));
for i = 1:length(t)
    %set desired foot pose for particular time step
    %x-coord is 0, y deterimined by Yfoot vector, and theta is -pi/2
    %(straight vertical)
    footpose = [0 Yfoot(i) -pi/2]';
    
    %compute all joint angles using subchainIK function
    angles = subchainIK(footpose, lengths, 0);
    
    %extract the values of the actuated joints 
    actuatedAngs(:,i) = angles(:,1);
end
actuatedAngs = actuatedAngs';
%compute actuated joint velocites
actuatedVels = diff(actuatedAngs) / dt;

%export to CSV file - to be imported into python/trep
csvwrite('JointVelocities.csv',actuatedVels);

% plot(t(1:end-1),actuatedVels)
