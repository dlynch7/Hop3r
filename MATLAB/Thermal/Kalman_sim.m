%% main function: Kalman_sim.m
function Kalman_sim()

close all;
clear;
clc;

T_amb = 20;
Tw_max = 125;
kv = 21.1/4.57; % stall current divided by stall torque. Close to datasheet's 4.6083 A/Nm
stand_torque = 1.5;
jump_torque = 4.6;
I_stand = stand_torque*kv;
I_jump = jump_torque*kv;
jump_period = 0.4;
jump_duty_cycle = 0.1;

internal_Kalman_option = 1;
ts = 0.1; % sampling period

params = init_params(T_amb,ts);

time_vector = 0:ts:180;
stand_control_vector = [I_stand^2; T_amb].*ones(2,length(time_vector));

for i = 1:length(time_vector)
    jump_control_vector(2,i) = T_amb;
    modulus = mod(time_vector(i),jump_period);
    if (modulus/jump_period < jump_duty_cycle)
        jump_control_vector(1,i) = I_jump^2;
    else
        jump_control_vector(1,i) = 0;
    end
end

% figure;
% plot(time_vector,jump_control_vector(1,:).^0.5,'k.-')
% xlabel('time (s)')
% ylabel('current (A)')
% title(['Current profile: jumping with period ',num2str(jump_period),...
%     ' s and duty cycle ',num2str(jump_duty_cycle),'.']);

% Simulate standing:
ICs = [T_amb; T_amb; T_amb];
[T, output, T_noisy, output_noisy, kalman_estimate] = simulator(ICs,...
    time_vector,stand_control_vector,params,...
    internal_Kalman_option);

figure
plot(time_vector,kalman_estimate(1,:),'b:'); hold on;
plot(time_vector,kalman_estimate(2,:),'g:');
plot(time_vector,kalman_estimate(3,:),'r:');
hL = plot(time_vector,T(1,:),'b-',time_vector,T(2,:),'g-',time_vector,T(3,:),'r-');
line([0 time_vector(end)],[Tw_max Tw_max],'Color','black')
legend(hL,'winding temp','housing temp','plate temp','Location','Best')
xlabel('time (s)')
ylabel('Temperature (deg C)')
title(['Simulated temperature profile for constant I = ',num2str(I_stand),' A'])
hold off

figure;
plot(time_vector,output_noisy,'g-',time_vector,kalman_estimate(2,:),'r-');
legend('measurement','estimate','Location','Best')
xlabel('time(s)')
ylabel('Temperature (deg C)')
title('Measurement vs Kalman filter estimate (standing)')

% Simulate jumping:
ICs = [T_amb; T_amb; T_amb];
[T, output, T_noisy, output_noisy, kalman_estimate] = simulator(ICs,...
    time_vector,jump_control_vector,params,...
    internal_Kalman_option);

figure
plot(time_vector,kalman_estimate(1,:),'b:'); hold on;
plot(time_vector,kalman_estimate(2,:),'g:');
plot(time_vector,kalman_estimate(3,:),'r:');
hL = plot(time_vector,T(1,:),'b-',time_vector,T(2,:),'g-',time_vector,T(3,:),'r-');
line([0 time_vector(end)],[Tw_max Tw_max],'Color','black')
legend(hL,'winding temp','housing temp','plate temp','Location','Best')
xlabel('time (s)')
ylabel('Temperature (deg C)')
title({['Simulated temperature profile: jumping (I = ',num2str(I_jump),...
    ' A)'],[' with period ',num2str(jump_period),...
    ' s and duty cycle ',num2str(jump_duty_cycle),'.']});
hold off

figure;
plot(time_vector,output_noisy,'g-',time_vector,kalman_estimate(2,:),'r-');
legend('measurement','estimate','Location','Best')
xlabel('time(s)')
ylabel('Temperature (deg C)')
title('Measurement vs Kalman filter estimate (jumping)')

end

%% Helper function: initialize thermal parameters and estimator params
function params = init_params(T_amb,t_sample)

% Calculate thermal "constants"
Ts = 100; % averaged Ts, very hand-wavey
g = 9.81;
beta = 3e-3; % https://www.engineeringtoolbox.com/air-density-specific-weight-d_600.html
L = 0.112; % characteristic length is surface area divided by perimeter
nu = 18.86e-6; % https://www.engineeringtoolbox.com/air-absolute-kinematic-viscosity-d_601.html
alpha = 27e-6; % https://www.engineeringtoolbox.com/air-thermal-diffusivity-d_2011.html
Pr = 0.705; % https://www.engineeringtoolbox.com/air-prandtl-number-viscosity-heat-capacity-thermal-conductivity-d_2009.html
RaL = (g*beta*(Ts - T_amb)*(L^3))/(nu*alpha);
NuL = (0.68 + ((0.67*(RaL^0.25))/((1 + ((0.492/Pr)^(9/16)))^(4/9))))^2;
kair = 28.8e-3; % https://www.engineeringtoolbox.com/air-properties-viscosity-conductivity-heat-capacity-d_1509.html?vA=60&degree=C&pressure=1bar#

R = 2.28;

mw = 0.3;
cw = 390;
Rtwh = 2.6;

mh = 0.3;
ch = 910;
Rthb = 1.91;

mb = 0.7;
cb = 910;
hb = NuL*kair/L;
Ab = 0.086;

% compute intermediate terms:
a = 1/(mw*cw*Rtwh);
b = 1/(mh*ch*Rtwh);
c = 1/(mh*ch*Rthb);
d = 1/(mb*cb*Rthb);
e = (hb*Ab)/(mb*cb);
f = R/(cw*mw);

% compute state-space model parameters:
A = [-a, a, 0;
    b, -b - c, c;
    0, d, -d - e];
B = [f,0;
    0,0;
    0,e];

params.state_space.state_matrix = A;
params.state_space.input_matrix = B;
params.state_space.output_matrix = [0, 1, 0];
params.state_space.feedthrough_matrix = [0, 0];

% compute estimator parameters:
params.estimator.process_noise_covariance = [5, 2.5, 1.25;
                                             2.5, 5, 2.5;
                                             1.25, 2.5, 5]; % made-up values
params.estimator.measurement_noise_covariance = 10; % made-up value

params.estimator.sampling_period = t_sample;
Ad = expm(A*t_sample);
params.estimator.discrete_state_matrix = Ad;
params.estimator.discrete_input_matrix = inv(A)*(Ad - eye(length(Ad)))*B;

params.estimator.prediction_covariance = eye(3); % initialization

end

%% Helper function: simulator
function [state, output, varargout] = simulator(ICs,time_vector,...
    control_vector,params,internal_Kalman_option)

    nout = max(nargout,1) - 2;
    
    state = zeros(length(ICs),length(time_vector)); % pre-allocation
    
    C = params.state_space.output_matrix; % should be 1x3
    D = params.state_space.feedthrough_matrix; % should be 1x2
    Q = params.estimator.process_noise_covariance; % should be 3x3
    R = params.estimator.measurement_noise_covariance; % should be a scalar
    
    state(:,1) = ICs;
    output(:,1) = C*state(:,1);
    
    process_noise = mvnrnd(zeros(1,length(Q)),Q)';
    measurement_noise = mvnrnd(0,R)';
    state_noisy(:,1) = state(:,1) + process_noise;
    output_noisy(:,1) = output(:,1) + measurement_noise;
    
    if internal_Kalman_option==1
        % compute 1st Kalman filter estimate:
        [kalman_filter_estimate(:,1),kalman_filter_prediction(:,1)] = ...
            kalman_filter(params,state(:,1),...
            output_noisy(:,1),control_vector(:,1));
    end
    
    for i = 2:length(time_vector)
        [time_dont_care, state_temp] = ode45(@(t,y)(thermal_dyn(y,control_vector(:,i),params)),...
            [time_vector(i-1), time_vector(i)],state(:,i-1)');
        
        state(:,i) = state_temp(end,:)';
        output(:,i) = C*state(:,i) + D*control_vector(:,i);
        
        process_noise = mvnrnd(zeros(1,length(Q)),Q)';
        measurement_noise = mvnrnd(0,R)';
        state_noisy(:,i) = state(:,i) + process_noise;
        output_noisy(:,i) = output(:,i) + measurement_noise;
        
        if internal_Kalman_option==1
            % compute i-th Kalman filter estimate:
            [kalman_filter_estimate(:,i),kalman_filter_prediction(:,i)] = ...
                kalman_filter(params,kalman_filter_estimate(:,i-1),...
                output_noisy(:,i),control_vector(:,i));
        end
    end
    
    if nout==0
        % do nothing
    elseif nout==2
        varargout{1} = state_noisy;
        varargout{2} = output_noisy;
    elseif (nout==3) && (internal_Kalman_option==1)
        varargout{1} = state_noisy;
        varargout{2} = output_noisy;
        varargout{3} = kalman_filter_estimate;
    elseif (nout==5) && (internal_Kalman_option==1)
        varargout{1} = state_noisy;
        varargout{2} = output_noisy;
        varargout{3} = kalman_filter_estimate;
        varargout{4} = kalman_filter_prediction;
        varargout{5} = output_noisy;
    else
        error(['Passed an unexpected number of output arguments: nout = ',...
            num2str(nout),'.']);
    end
end

%% Helper function: thermal system dynamics
function [statedot] = thermal_dyn(state,control,params)

% extract individual temperatures from state vector:
Tw = state(1); % winding temperature    [C]
Th = state(2); % housing temperature    [C]
Tb = state(3); % backplate temperature  [C]

% extract inputs to thermal model from control vector:
I_squared = control(1); % winding current squared   [A^2]
T_amb = control(2);      % ambient temperature       [C]

% extract state and input matrices of state-space model:           
A = params.state_space.state_matrix;
B = params.state_space.input_matrix;

% compute time derivative of state:
statedot = A*[Tw; Th; Tb] + B*[I_squared; T_amb];
            
end

%% Helper function: Kalman filter
function [kalman_filter_estimate,kalman_filter_prediction] = kalman_filter(...
            params,prev_estimate,measurement,control)
   
    % extract parameters:
    P_prev = params.estimator.prediction_covariance;
    Ad = params.estimator.discrete_state_matrix;
    Bd = params.estimator.discrete_input_matrix;
    C = params.state_space.output_matrix;
    Q = params.estimator.process_noise_covariance;
    R = params.estimator.measurement_noise_covariance;
    
    % prediction step:
    kalman_filter_prediction = Ad*prev_estimate + Bd*control;
    P_a_priori = Ad*P_prev*(Ad') + Q;
   
    % update step:
    kalman_gain = P_a_priori*(C')*inv(C*P_a_priori*(C') + R);
    kalman_filter_estimate = kalman_filter_prediction...
        + kalman_gain*(measurement - (C*kalman_filter_prediction));
    P_a_posteriori = (eye(length(Ad)) - kalman_gain*C)*P_a_priori;
    params.estimator.prediction_covariance = P_a_posteriori;
    
end