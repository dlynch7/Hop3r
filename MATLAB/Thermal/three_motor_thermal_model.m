%% three_motor_thermal_model.m
% Dan Lynch
% 3/16/18

%% Description

%%
clear;
close all;
clc;

%% Calculate thermal "constants"
Ts = 100; % averaged Ts, very hand-wavey
Tamb = 20;
g = 9.81;
beta = 3e-3; % https://www.engineeringtoolbox.com/air-density-specific-weight-d_600.html
L = 0.112; % characteristic length is surface area divided by perimeter
nu = 18.86e-6; % https://www.engineeringtoolbox.com/air-absolute-kinematic-viscosity-d_601.html
alpha = 27e-6; % https://www.engineeringtoolbox.com/air-thermal-diffusivity-d_2011.html
Pr = 0.705; % https://www.engineeringtoolbox.com/air-prandtl-number-viscosity-heat-capacity-thermal-conductivity-d_2009.html
RaL = (g*beta*(Ts - Tamb)*(L^3))/(nu*alpha);
NuL = (0.68 + ((0.67*(RaL^0.25))/((1 + ((0.492/Pr)^(9/16)))^(4/9))))^2;
kair = 28.8e-3; % https://www.engineeringtoolbox.com/air-properties-viscosity-conductivity-heat-capacity-d_1509.html?vA=60&degree=C&pressure=1bar#

%% Parameters
Tw_max = 125;

kv = 21.1/4.57; % stall current divided by stall torque. Close to datasheet's 4.6083 A/Nm
torque = 1.5;

I = torque*kv;
R = 2.28;

mw = 0.3;
cw = 390;
Rtwh = 2.6;

mh = 0.3;
ch = 910;
Rthp = 1.91;

mAL = 0.7;
cAL = 910;
hAL = NuL*kair/L;
AAL = 0.086;

%% system matrix A (state -> statedot):
% row 1:
A11 = -1/(mw*cw*Rtwh);
A12 = 1/(mw*cw*Rtwh);
A13 = 0; A14 = 0; A15 = 0; A16 = 0;
A17 = 0;

% row 2:
A21 = 1/(mh*ch*Rtwh);
A22 = -1/(mh*ch*Rtwh) - 1/(mh*ch*Rthp);
A23 = 0; A24 = 0; A25 = 0; A26 = 0;
A27 = 1/(mh*ch*Rthp);

% row 3:
A31 = 0; A32 = 0;
A33 = -1/(mw*cw*Rtwh);
A34 = 1/(mw*cw*Rtwh);
A35 = 0; A36 = 0; A37 = 0;

% row 4:
A41 = 0; A42 = 0;
A43 = 1/(mh*ch*Rtwh);
A44 = -1/(mh*ch*Rtwh) - 1/(mh*ch*Rthp);
A45 = 0; A46 = 0;
A47 = 1/(mh*ch*Rthp);

% row 5:
A51 = 0; A52 = 0; A53 = 0; A54 = 0;
A55 = -1/(mw*cw*Rtwh);
A56 = 1/(mw*cw*Rtwh);
A57 = 0;

% row 6:
A61 = 0; A62 = 0; A63 = 0; A64 = 0;
A65 = 1/(mh*ch*Rtwh);
A66 = -1/(mh*ch*Rtwh) - 1/(mh*ch*Rthp);
A67 = 1/(mh*ch*Rthp);

% row 7:
A71 = 0;
A72 = 1/(mAL*cAL*Rthp);
A73 = 0;
A74 = 1/(mAL*cAL*Rthp);
A75 = 0;
A76 = 1/(mAL*cAL*Rthp);
A77 = -((3/mAL*cAL*Rthp) + ((hAL*AAL)/(mAL*cAL)));

A = [A11 A12 A13 A14 A15 A16 A17;
     A21 A22 A23 A24 A25 A26 A27;
     A31 A32 A33 A34 A35 A36 A37;
     A41 A42 A43 A44 A45 A46 A47;
     A51 A52 A53 A54 A55 A56 A57;
     A61 A62 A63 A64 A65 A66 A67;
     A71 A72 A73 A74 A75 A76 A77];
 
%% weighted input vector (Bu):
I1 = I; I2 = I; I3 = I;

B1 = (I1^2)*R;
B2 = 0;
B3 = (I2^2)*R;
B4 = 0;
B5 = (I3^2)*R;
B6 = 0;
B7 = ((hAL*AAL)/(mAL*cAL))*Tamb;

Bu = [B1; B2; B3; B4; B5; B6; B7];

%% constant current model:
dTdtDC = @(t,T) A*T + Bu;

%% Simulate, for steady-state current draw:
T_init = [20; 20; 20; 20; 20; 20; 20];
timespan = [0 180];
[t,T] = ode45(dTdtDC,timespan,T_init);

%% plot steady-state simulation result:
plot(t,T(:,7))