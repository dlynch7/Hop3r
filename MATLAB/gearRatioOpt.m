%% gearRatioOpt.m
% Author: Dan Lynch
% Start Date: 2-14-2018

%% Description
% Find the optimal gear ratio that:
%   maximizes output torque
%   maximizes torque density
%   maximizes backdrivability
%   drives the ratio of reflected inertia to motor inertia to 1 (ideal)

%% main function
function gearRatioOpt
    close all;
    clear;
    clc;
    
    %% Global variables
    global inputTorque;
    global motorInertia;
    global loadInertia;
    global motorMass;

    global outputTorqueWeight;
    global torqueDensityWeight;
    global backdrivabilityWeight;
    global inertiaRatioWeight;

    %% Constants
    inputTorque = 4.94; % Nm
    motorInertia = 3060*(1/1000)*(1/100)^2;
    loadInertia = 0.0001;
    motorMass = 1; % kg

    %% User-defined weights
    outputTorqueWeight = 10;
    torqueDensityWeight = 1;
    backdrivabilityWeight = 1;
    inertiaRatioWeight = 1;
    
    %% Variable being optimized: gear ratio
    N = linspace(1,100,1000);
    
    %% output
    output = zeros(1,length(N));
    for i = 1:length(N)
        output(i) = objectiveFunction(N(i));
    end
    
    [minimum, minimizer] = min(output);
    minN = N(minimizer);
    
    fprintf('Optimal gear ratio:\t%f\n',minN);
    fprintf('Optimum:\t\t%f\n',minimum);
    fprintf('\n');
    fprintf('Output torque:\t\t%f\n',outputTorque(inputTorque, minN))
    fprintf('Torque Density:\t\t%f\n',torqueDensity(inputTorque,minN))
    fprintf('Backdrivability:\t%f\n',backdrivability(minN))
    fprintf('Inertia Ratio:\t\t%f\n',inertiaRatio(minN,motorInertia,loadInertia))
    fprintf('\n')
    
    %% plot
    plot(N,output);
    title(['weights: ',num2str(outputTorqueWeight),', ',...
        num2str(torqueDensityWeight),', ',num2str(backdrivabilityWeight),...
        ', ',num2str(inertiaRatioWeight)]);
end

%% Objective function
function J = objectiveFunction(gearRatio)
    global inputTorque;
    global motorInertia;
    global loadInertia;

    global outputTorqueWeight;
    global torqueDensityWeight;
    global backdrivabilityWeight;
    global inertiaRatioWeight;
    
    J = outputTorqueWeight*(20 - outputTorque(inputTorque, gearRatio))^2 ...
        + torqueDensityWeight*(1/torqueDensity(inputTorque,gearRatio))^2 ...
        + backdrivabilityWeight*(1/backdrivability(gearRatio))^2 ...
        + inertiaRatioWeight*(1 - inertiaRatio(gearRatio,motorInertia,loadInertia))^2;
    
end

%% calculate output torque
function Tout = outputTorque(Tin, N)
    Tout = Tin*N;
end

%% calculate torque density
function Td = torqueDensity(Tin,N)
    global motorMass;
    Tout = outputTorque(Tin,N);
    GM = gearMass(N);
    Td = Tout/(GM + motorMass);
end

%% calculate backdrivability
function B = backdrivability(N)
    B = 1/N;
end

%% calculate ratio of motor inertia to reflected inertia
function IR = inertiaRatio(N,Jm,Jl)
    reflectedInertia = Jl*(N^2); %  motor inertia reflected through gearing
    IR = Jm/reflectedInertia;
end

%% calculate gearhead mass
function GM = gearMass(N)
    kg_per_N = 0.0064; % very hand-wavey, based on Maxon 223081, 223084, 223090
    GM = kg_per_N*N;
end