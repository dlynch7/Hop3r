%% %% two_mass_thermal_model.m
%   Dan Lynch
%   3/8/18
%% Description

%% Parameters
tau_w = 46; % thermal time constant of motor windings (s)
% tau_h = 283; % thermal time constant of motor housing (s)
tau_h = 500; % thermal time constant of motor housing (s)
tau_wh = mean([tau_w tau_h]); % (GUESS) thermal time constant, mixed winding/housing (s)
cp_w = 386; % windings (copper) specific heat (J/kg-K)
m_w = 0.3; % winding mass (kg)

R = 0.343; % terminal resistance (ohms)
T_amb = 20; % ambient temperature (deg C)

Tw_max = 125; % max winding temperature (deg C)

%% ODE
dTdt = @(t,T,I) [-1/tau_w, 1/tau_w; 
                1/tau_wh, -(1/tau_wh)+(1/tau_h)]*[T(1); T(2)] + [(I^2)*R/(cp_w*m_w); T_amb/tau_h];
            
%% Simulate
sim_t = 0;
sim_T = [0 0];

on_time = 0.1;
off_time = 0.4 - on_time;
while sim_t(end) < 60
% heating:
I = 70;
dTdt_heating = @(t,T) dTdt(t,T,I);
T_init = [sim_T(end,1); sim_T(end,2)]; % everything starts at ambient temperature
timespan = [sim_t(end) sim_t(end)+on_time];

[t_heating,T_heating] = ode45(dTdt_heating,timespan,T_init);

% cooling:
I = 0;
dTdt_cooling = @(t,T) dTdt(t,T,I);
T_init = [T_heating(end,1); T_heating(end,2)]; % everything starts at ambient temperature
timespan = [sim_t(end)+on_time sim_t(end)+on_time+off_time];

[t_cooling,T_cooling] = ode45(dTdt_cooling,timespan,T_init);

t = vertcat(t_heating, t_cooling);
T = vertcat(T_heating, T_cooling);

sim_t = vertcat(sim_t, t);
sim_T = vertcat(sim_T, T);

end
figure

plot(sim_t,T_amb+sim_T(:,1),'b-',sim_t,T_amb+sim_T(:,2),'g-')
line([0 sim_t(end)],[Tw_max-T_amb Tw_max-T_amb],'Color','red')
legend('winding temp','housing temp','Location','Best')
xlabel('time (s)')
ylabel('Temperature (deg C)')
hold off 