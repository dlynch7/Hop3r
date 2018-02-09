clear;
close all;
clc;
%% test forward kinematics algorithm

%% parameters
L1 = 1;
L2 = 1;
L3 = 0.8;
L4 = 0.9;
L5 = 1;
L6 = 2;
L7 = 0.75;
L8 = 1.8;
B1 = 1;
B2 = 1;
lengths = [L1; L2; L3; L4; L5; L6; L7; L8; B1; B2];

%% input joint angles
theta1d = -135; % degrees
phi1d = -135; % degrees
psi1d = -45; % degrees

% convert to radians:
theta1 = (pi/180)*theta1d;
phi1 = (pi/180)*phi1d;
psi1 = (pi/180)*psi1d;

% combine angles into vector:
qa = [theta1; phi1; psi1];

%% initial guess: passive joint angles
theta2 = (pi/180)*90;
theta3 = (pi/180)*(-45);
phi2 = (pi/180)*90;
phi3 = (pi/180)*(-45);
psi2 = (pi/180)*(-90);
psi3 = (pi/180)*45;

% combine angles into vector:
qu0 = [theta2; theta3; phi2; phi3; psi2; psi3];

%% numerically solve forward kinematics
thresh = 0.01;
chain = 1; % theta-chain
footPose = NRFK(qa,qu0,lengths,thresh,chain);