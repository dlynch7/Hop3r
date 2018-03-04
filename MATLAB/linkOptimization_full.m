%%
clear;
close all;
clc;

%% Define minimum and maximum lengths for each link:
L1min = 0.05;
L1max = 0.15;

L2min = 0.05;
L2max = 0.15;

L3min = 0.05;
L3max = 0.15;

L4min = 0.05;
L4max = 0.15;

L5min = 0.05;
L5max = 0.15;

L6min = 0.05;
L6max = 0.15;

L7min = 0.05;
L7max = 0.15;

L8min = 0.05;
L8max = 0.15;

B1xmin = 0.05;
B1xmax = 0.15;

B2xmin = 0.05;
B2xmax = 0.15;

B1ymin = 0.05;
B1ymax = 0.15;

B2ymin = 0.05;
B2ymax = 0.15;

linkMin = [L1min; L2min; L3min; L4min; L7min; L8min; B1xmin; B1ymin];
linkMax = [L1max; L2max; L3max; L4max; L7max; L8max; B1xmax; B1ymax];

%% Specify desired end-effector wrench in end-effector frame:
Fx = 71.5; % force directed distally along link
Fy = 0;
Mz = 0;

ee_wrench = [Fx; Fy; Mz];

%% Use fmincon to optimize link lengths:

% objective function weights:
torqueWeight = 100;
workspaceVolWeight = 1;

% create anonymous function:
f = @(x)dimensionObjectiveFunction(x,ee_wrench,torqueWeight,workspaceVolWeight);

% initial guess:
x0 = [0.0762; 0.1207; 0.0762; 0.0762; 0.0508; 0.1500; 0.1016; 0.02];

% no linear constraints:
A = [];
b = [];
Aeq = [];
beq = [];

% bounds
lb = linkMin;
ub = linkMax;
linkOpt = fmincon(f,x0,A,b,Aeq,beq,lb,ub)