function minimizationVec = dimensionOptimization()
% Compute the "optimal" robot dimensions - treating the center motor as the
% origin, find the position (x,y) of the side motors and the side chain
% link lengths that minimize the torque required for jumping.
% see dimensionOptimization.pdf for diagrams and variable definitions
% Output: minimizationVec - values of the minimal torque configuration
% = [Tmin, xmin, ymin, amin, bmin]';

%Energy required to lift 5kg to h = 10cm
E = 5 * 9.81 * 0.1;
E = E/2; %symmetry - motor on one side responsible for 50% of total energy

d = 5; %the end of link b's initial distance below origin

%define vectors to be tested
xvec = linspace(1,15,10);
yvec = linspace(-10,5,10);
avec = linspace(5,25,10);
bvec = linspace(5,25,10);

%initialize minimization variables
Tmin = inf;
xmin = 0;
ymin = 0;
amin = 0;
bmin = 0;

%perform minimization
for x = xvec
    for y = yvec
        for a = avec
            for b = bvec
                
                try
                    [t1,t2] = IK(x,y,a,b,d);
                    
                    tf = asin(x / (a+b));
                    
                    angular_travel = t1 + tf;
                    vertical_travel = sqrt((a+b)^2 - x^2) - y - d;
                    
                    %Assume constant force through vertical travel
                    %Work = change in energy = Force*distance
                    F = E / (vertical_travel/100); %convert vertical travel from cm to meters for this calculation
                    
                    %This is the big assumption that probably is invalid
                    %Probably should use actuatorJacobian here
                    T = F * (x/100); %assume constant torque
                    if T < Tmin
                        Tmin = T;
                        xmin = x;   ymin = y;   amin = a;   bmin = b;
                    end
                    
                catch
                    T = inf;
                end
                
            end
        end
    end
end

%return minimizer
minimizationVec = [Tmin, xmin, ymin, amin, bmin]';









function [t1,t2] = IK(x,y,a,b,d)
%Inverse Kinematics helper function
%use law of cosines to solve angles - see dimensionOptimization.pdf for diagrams
%and angle definitions

%Inputs: x and y: motor's (x,y) position relative to center motor 'origin'
%        a and b: link lengths
%        d: starting point height below origin
%Outputs: t1: angle of link 'a' relative to the the vertical
%         t2: angle of link 'b' relative to the horizontal
%
% The end of the 'b' link is constrained to the robot centerline (x coord = 0)
% All lengths in centimeters

% Calculate hip angle:

gamma = wrapToPi(abs(atan2(abs(y+d),x)));
alpha = wrapToPi(acos((x^2 + (y+d)^2 + a^2 - b^2)/(2*a*sqrt(x^2 + (y+d)^2))));
t1 = wrapToPi(gamma + alpha - pi/2);

% Calculate (x,y) location of "knee" joint:
xK = x + a*sin(t1);
yK = y - a*cos(t1);

% Calculate knee angle:
t2 = atan2(yK+d, xK);


