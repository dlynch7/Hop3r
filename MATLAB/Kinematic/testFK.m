% clear;
% close all;
% clc;
%% test forward kinematics algorithm

%% parameters
L1 = 76.2/1000;
L2 = 120.65/1000;
L3 = 76.2/1000;
L4 = 76.2/1000;
L5 = L1;
L6 = L2;
L7 = 50.8/1000;
L8 = 165.1/1000;
L9 = 101.6/1000;
L10 = 101.6/1000;
L11 = 20/1000;
L12 = 20/1000;

lengths = [L1; L2; L3; L4; L5; L6; L7; L8; L9; L10; L11; L12];

%% input joint angles
% theta1d = -135; % degrees
% phi1d = -135; % degrees
% psi1d = -45; % degrees
% 
% % convert to radians:
% theta1 = (pi/180)*theta1d;
% phi1 = (pi/180)*phi1d;
% psi1 = (pi/180)*psi1d;

theta1 = -1.6244; % -93.0687*(pi/180)
phi1 = -2.7118; % -155.3757*(pi/180)
psi1 = -1.5172; % -86.9313*(pi/180)

% combine angles into vector:
qaF = [theta1; phi1; psi1];

%% initial guess: passive joint angles
theta2 = 1.1209;
theta3 = -1.0673;
phi2 = 2.2820;
phi3 = -1.1410;
psi2 = -1.1209;
psi3 = 1.0673;

% theta2 = 64.2223*(pi/180); % 64.2223*(pi/180)
% theta3 = -61.1536*(pi/180); % -61.1536*(pi/180)
% 
% phi2 = 130.7514*(pi/180); % 130.7514*(pi/180)
% phi3 = -65.3757*(pi/180); % -65.3757*(pi/180)
% 
% psi2 = -64.2223*(pi/180); % -64.2223*(pi/180)
% psi3 = 61.1536*(pi/180); % 61.1536*(pi/180)

% combine angles into vector:
qu0 = [theta2; theta3; phi2; phi3; psi2; psi3];

%% numerically solve forward kinematics
thresh = 0.01;
chain = 1; % theta-chain
footPose = NRFK(qaF,qu0,lengths,thresh,chain);