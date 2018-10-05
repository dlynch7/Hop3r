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
            
%% Simulate, for steady-state current draw:
T_init = [20; 20; 20];
timespan = [0 180];
[t,T] = ode45(dTdt,timespan,T_init);

%% Plot simulation result:
figure
plot(t,T(:,1),'b-',t,T(:,2),'g-',t,T(:,3),'r-')
hold on
line([0 t(end)],[Tw_max Tw_max],'Color','black')
legend('winding temp','housing temp','plate temp','Location','Best')
xlabel('time (s)')
ylabel('Temperature (deg C)')
title(['Simulated temperature profile for constant I = ',num2str(I),' A'])
hold off 

%% Simulate, for current needed to jump:
torqueJump = 4.6;

sim_t = 0;
sim_T = [0 0 0];

on_time = 0.1;
off_time = 0.4 - on_time;
while sim_t(end) < 180
% heating:
I = torqueJump*kv;
dTdt = @(t,T) [-1/(mw*cw*Rtwh), 1/(mw*cw*Rtwh), 0; 
                1/(mh*ch*Rtwh), -1/(mh*ch*Rtwh) - 1/(mh*ch*Rtha), 1/(mh*ch*Rtha);
                0, 1/(mAL*cAL*Rtha), -1/(mAL*cAL*Rtha) - (hAL*AAL)/(mAL*cAL)]...
                *[T(1); T(2); T(3)]...
                + [(I^2)*R/(cw*mw); 0; ((hAL*AAL)/(mAL*cAL))*Tamb];
dTdt_heating = @(t,T) dTdt(t,T);
T_init = [sim_T(end,1); sim_T(end,2); sim_T(end,3)]; % everything starts at ambient temperature
timespan = [sim_t(end) sim_t(end)+on_time];

[t_heating,T_heating] = ode45(dTdt_heating,timespan,T_init);

% cooling:
I = 0;
dTdt = @(t,T) [-1/(mw*cw*Rtwh), 1/(mw*cw*Rtwh), 0; 
                1/(mh*ch*Rtwh), -1/(mh*ch*Rtwh) - 1/(mh*ch*Rtha), 1/(mh*ch*Rtha);
                0, 1/(mAL*cAL*Rtha), -1/(mAL*cAL*Rtha) - (hAL*AAL)/(mAL*cAL)]...
                *[T(1); T(2); T(3)]...
                + [(I^2)*R/(cw*mw); 0; ((hAL*AAL)/(mAL*cAL))*Tamb];
dTdt_cooling = @(t,T) dTdt(t,T);
T_init = [T_heating(end,1); T_heating(end,2); T_heating(end,3)]; % everything starts at ambient temperature
timespan = [sim_t(end)+on_time sim_t(end)+on_time+off_time];

[t_cooling,T_cooling] = ode45(dTdt_cooling,timespan,T_init);

t = vertcat(t_heating, t_cooling);
T = vertcat(T_heating, T_cooling);

sim_t = vertcat(sim_t, t);
sim_T = vertcat(sim_T, T);

end
figure

plot(sim_t,sim_T(:,1),'b-',sim_t,sim_T(:,2),'g-',sim_t,sim_T(:,3),'r-')
line([0 sim_t(end)],[Tw_max Tw_max],'Color','black')
legend('winding temp','housing temp','plate temp','Location','Best')
xlabel('time (s)')
ylabel('Temperature (deg C)')
title(['Simulated temperature profile for jumping, I = ',num2str(torqueJump*kv),' A'])
hold off 

%% Check stability and observability
A = [-1/(mw*cw*Rtwh), 1/(mw*cw*Rtwh), 0; 
                1/(mh*ch*Rtwh), -1/(mh*ch*Rtwh) - 1/(mh*ch*Rtha), 1/(mh*ch*Rtha);
                0, 1/(mAL*cAL*Rtha), -1/(mAL*cAL*Rtha) - (hAL*AAL)/(mAL*cAL)];

fprintf('Eigenvectors (V) and eigenvalues (D) of system matrix A:\n');
[V,D] = eig(A)
            
C1 = [0 0 0;
      0 1 0;
      0 0 1];

C2 = [0 0 0;
      0 1 0;
      0 0 0];

O1 = obsv(A,C1);
O2 = obsv(A,C2);

fprintf("rank(O1) = %d\n",rank(O1));
fprintf("rank(O2) = %d\n",rank(O2));