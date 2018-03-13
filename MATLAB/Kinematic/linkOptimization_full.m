%%
clear;
% close all;
% clc;

%% TO-DO:
% 0. reduce range of foot angles in searchable workspace.
% 1. Compute torque of a subset of the searchable foot angles.
% 2. Plot point cloud of reachable workspace, check for contiguity.
% 3. Generage mesh from point cloud, compute surface area (or volume?) of
% mesh.
% 4. Do different initial guesses converge to the same optimizer?
% 5. If dependence on initial guess is significant, try simulated annealing
% in scipy.
% 6. Consider reachable workspace as constraint rather than variable.

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

B1ymin = 0;
B1ymax = 0.15;

B2ymin = 0;
B2ymax = 0.15;

linkMin = [L1min; L2min; L3min; L4min; L7min; L8min; B1xmin; B1ymin];
linkMax = [L1max; L2max; L3max; L4max; L7max; L8max; B1xmax; B1ymax];

%% Specify desired end-effector wrench in end-effector frame:
Fx = 75.0; % force directed distally along link
Fy = 0;
Mz = 0;

ee_wrench = [Fx; Fy; Mz];

%% Optimize link lengths:

% objective function weights:
torqueWeight = 100;
workspaceVolWeight = 1;

% resolution of searched workspace:
rez = 11;

% create anonymous function:
f = @(x)dimensionObjectiveFunction(x,ee_wrench,torqueWeight,workspaceVolWeight,rez,0);

% initial guess:
% x0 = [0.0762; 0.1207; 0.0762; 0.0762; 0.0508; 0.1500; 0.1016; 0.02];
x0 = [0.0762; 0.1207; 0.0762; 0.0762; 0.0508; 0.1500; 0.1016; 0.02];

% no linear constraints:
A = [];
b = [];
Aeq = [];
beq = [];

% bounds
lb = linkMin;
ub = linkMax;

% optimize link lengths using fmincon():
[linkOpt_fmc,fval_fmc,exitflag_fmc,output_fmc] = fmincon(f,x0,A,b,Aeq,beq,lb,ub);
fprintf('fmincon() optimization results: \n');
fprintf('The number of iterations was : %d\n', output_fmc.iterations);
% fprintf('The number of function evaluations was : %d\n', output_fmc.funccount);
fprintf('The best function value found was : %g\n', fval_fmc);
fprintf('optimal link lengths: \n');
linkOpt_fmc

%% Visualize reachable workspace of optimal link lengths:
hirez = 51;
J_opt = dimensionObjectiveFunction(linkOpt_fmc,ee_wrench,torqueWeight,workspaceVolWeight,hirez,1);