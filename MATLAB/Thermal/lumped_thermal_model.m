%% Lumped thermal model
clear;
clc;

%% Parameters
tau_m = 283; % thermal time constant of motor housing (s)
cp_w = 386; % windings (copper) specific heat (J/kg-K)
m_w = 0.6/2; % winding mass (kg)

R = 2.28; % terminal resistance (ohms)
T_amb = 20; % ambient temperature (deg C)

Tw_max = 125; % max winding temperature (deg C)

%% ODE
dTdt = @(t,T,I) (-1/tau_m)*(T - T_amb) + (R/(m_w*cp_w))*(I^2);
            
%% Simulate
sim_t = 0;
sim_T = 0;

on_time = 0.1;
off_time = 0.4 - on_time;
while sim_t(end) < 60
    % heating:
    I = 21.1;
    dTdt_heating = @(t,T) dTdt(t,T,I);
    T_init = sim_T(end); % everything starts at ambient temperature
    timespan = [sim_t(end) sim_t(end)+on_time];

    [t_heating,T_heating] = ode45(dTdt_heating,timespan,T_init);

    % cooling:
    I = 0;
    dTdt_cooling = @(t,T) dTdt(t,T,I);
    T_init = T_heating(end); % everything starts at ambient temperature
    timespan = [sim_t(end)+on_time sim_t(end)+on_time+off_time];

    [t_cooling,T_cooling] = ode45(dTdt_cooling,timespan,T_init);

    t = vertcat(t_heating, t_cooling);
    T = vertcat(T_heating, T_cooling);

    sim_t = vertcat(sim_t, t);
    sim_T = vertcat(sim_T, T);

end
figure

plot(sim_t,T_amb+sim_T,'b-')
line([0 sim_t(end)],[Tw_max-T_amb Tw_max-T_amb],'Color','red')
xlabel('time (s)')
ylabel('Temperature above ambient(deg C)')
title('Maxon 244879, 25% duty cycle')
hold off 