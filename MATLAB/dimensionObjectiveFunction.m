function J = dimensionObjectiveFunction(lengths, wrench, torqueWeight, volWeight)
%% Description:
% Computes a quadratic objective function (for optimization) using torques
% and joint velocities, calculated from an input wrench and an input twist
% using the actuator Jacobian (which is a function of link lengths and
% joint angles).

% Inputs:
% lengths: 12x1 array
%   [L1; L2; L3; L4; L5; L6; L7; L8; B1x; B1y; B2x; B2y];

% wrench: 3x1 array
%   [Fx; Fy; Mz];

% torqueWeight: scalar weight

% volWeight: scalar weight

% Output:
% J: scalar value of the objective function

%% Define the end-effector workspace that will be searched for the "achievable" workspace:
% footXarray = linspace(-0.25, 0.25, 11);
footXarray = linspace(-0.05, 0.05, 11);
% footYarray = linspace(-0.5, 0.0, 11);
footYarray = linspace(-0.25, -0.25, 11);
% footAnglearray = linspace(-pi/4, -3*pi/4, 11);
footAnglearray = linspace(-pi/2, -pi/2, 11);

%% Brute-force exploration of workspace:
for i = 1:length(footXarray)
    for j = 1:length(footYarray)
        for k = 1:length(footAnglearray)
            try
                footPose = [footXarray(i); footYarray(j); footAnglearray(k)];
                [tempAngles] = subchainIK(footPose,lengths,0);
                if 1
                %                 if checkJointLimits(tempAngles)
                    anglesD{i,j,k} = tempAngles.*(180/pi);
                    plotRobot(footPose,lengths,tempAngles);
                    fprintf(['passed: x = ',num2str(footXarray(i)),', y = ',...
                        num2str(footYarray(j)),', ang = ',num2str(footAnglearray(k)),'\n']);
                else
                    anglesD{i,j,k} = NaN;
                    fprintf(['failed: x = ',num2str(footXarray(i)),', y = ',...
                        num2str(footYarray(j)),', ang = ',...
                        num2str(footAngleDarray(k)),' exceeds joint limits.\n']);
                end
            catch
                anglesD{i,j,k} = NaN;
                fprintf(['failed: x = ',num2str(footXarray(i)),', y = ',...
                        num2str(footYarray(j)),', ang = ',...
                        num2str(footAnglearray(k)),' is unsolvable.\n']);
            end % end try/catch
        end
    end
end

%% Calculate joint torques (Nm) from input wrench:
torques = wrench2torques(angles, lengths, wrench);

%% Calculate value of objective function:
J = torqueWeight*(torqueMax^2) + volWeight*(volWorkspace^2);
end