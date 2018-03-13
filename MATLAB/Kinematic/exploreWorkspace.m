%% exploreWorkspace.m
%
% Description: systematically explores the robot's workspace

clear;
close all;
clc;

%% Parameters:
L1 = 1; % theta-chain
L2 = 1.5; % theta-chain
L3 = 0.75; % phi-chain
L4 = 1; % phi-chain
L5 = L1; % theta-chain and psi-chain are symmetric
L6 = L2; % theta-chain and psi-chain are symmetric
L7 = 0.5;
L8 = 1.5;
B1x = 1;
B2x = B1x; % theta-chain and psi-chain are symmetric
B1y = 0;
B2y = B1y; % theta-chain and psi-chain are symmetric
lengths = [L1 L2 L3 L4 L5 L6 L7 L8 B1x B2x B1y B2y]; % link lengths

%% Explore workspace:
footXarray = -1:0.5:1;
footYarray = -3.5:0.5:-2;
footAngleDarray = -100:5:-80;
anglesD = cell(length(footXarray),length(footYarray),length(footAngleDarray));

for i = 1:length(footXarray)
    for j = 1:length(footYarray)
        for k = 1:length(footAngleDarray)
            try
                footPose = [footXarray(i); footYarray(j); footAngleDarray(k)*(pi/180)];
                [tempAngles] = subchainIK(footPose,lengths,0);
%                 if checkJointLimits(tempAngles)
                if 1
                    anglesD{i,j,k} = tempAngles.*(180/pi);
                    plotRobot(footPose,lengths,tempAngles);
                    fprintf(['passed: x = ',num2str(footXarray(i)),', y = ',...
                        num2str(footYarray(j)),', ang = ',num2str(footAngleDarray(k)),'\n']);
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
                        num2str(footAngleDarray(k)),' is unsolvable.\n']);
            end % end try/catch
        end
    end
end