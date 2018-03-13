function withinLimits = checkJointLimits(angles)
load('jointLimits.mat');

%% "unpack" angles:
theta1 = angles(1,1);
theta2 = angles(1,2);
theta3 = angles(1,3);

phi1 = angles(2,1);
phi2 = angles(2,2);
phi3 = angles(2,3);

psi1 = angles(3,1);
psi2 = angles(3,2);
psi3 = angles(3,3);

%% check angles against limits:
if ((theta1 > theta1min) && (theta1 < theta1max) && ...
        (theta2 > theta2min) && (theta2 < theta2max) ...
        && (theta3 > theta3min) && (theta3 < theta3max))
    if ((phi1 > phi1min) && (phi1 < phi1max) && ...
            (phi2 > phi2min) && (phi2 < phi2max) ...
            && (phi3 > phi3min) && (phi3 < phi3max))
        if ((psi1 > psi1min) && (psi1 < psi1max) && ...
                (psi2 > psi2min) && (psi2 < psi2max) ...
                && (psi3 > psi3min) && (psi3 < psi3max))
            withinLimits = 1; % all joint angles are within limits
        else
            withinLimits = 0; % a psi-chain angle is out of range
        end
    else
        withinLimits = 0; % a phi-chain angle is out of range
    end
else
    withinLimits = 0; % a theta-chain angle is out of range
end

%% check for singularity at ankle:
if ((theta3 - psi3) < -pi)
    withinLimits = 0;
end

end