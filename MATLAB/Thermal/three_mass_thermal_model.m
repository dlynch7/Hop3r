%% %% three_mass_thermal_model.m
%   Dan Lynch
%   3/15/18
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
Rtha = 1.91;

mAL = 0.7;
cAL = 910;
hAL = NuL*kair/L;
AAL = 0.086;

%% ODE
dTdt = @(t,T) [-1/(mw*cw*Rtwh), 1/(mw*cw*Rtwh), 0; 
                1/(mh*ch*Rtwh), -1/(mh*ch*Rtwh) - 1/(mh*ch*Rtha), 1/(mh*ch*Rtha);
                0, 1/(mAL*cAL*Rtha), -1/(mAL*cAL*Rtha) - (hAL*AAL)/(mAL*cAL)]...
                *[T(1); T(2); T(3)]...
                + [(I^2)*R/(cw*mw); 0; ((hAL*AAL)/(mAL*cAL))*Tamb];
            
%% Simulate
T_init = [Tamb; Tamb; Tamb];
timespan = [0 120];
[t,T] = ode45(dTdt,timespan,T_init);

%% Plot simulation result:
figure
plot(t,Tamb+T(:,1),'b-',t,Tamb+T(:,2),'g-',t,Tamb+T(:,3),'r-')
hold on
line([0 t(end)],[Tw_max-Tamb Tw_max-Tamb],'Color','black')
legend('winding temp','housing temp','plate temp','Location','Best')
xlabel('time (s)')
ylabel('Temperature (deg C)')
hold off 