clear;
clc;
close all;

%% ========================================================================
%  Initialization
%  ========================================================================

addpath(fileparts(fileparts(mfilename('fullpath'))));

p = limo.parameters();
c = limo.control_parameters();


%% ========================================================================
%  Simulation parameters
%  ========================================================================

T = 10;                         % [s]
Ts = c.sample_time;             % [s]

t = 0:Ts:T;
N = length(t);


%% ========================================================================
%  Reference
%  ========================================================================

v_d = 0;                        % Initial velocity [m/s]
delta_d = 0;                    % Steering angle [rad]

u_c = [v_d; delta_d];


%% ========================================================================
%  Initial conditions
%  ========================================================================

% State:
%
% x = [X; Y; psi;
%      v;
%      omega_FL; omega_FR; omega_RL; omega_RR;
%      delta_L; delta_R;
%      delta_dot_L; delta_dot_R]

x = zeros(12,1);

% Controller state:
%
% xc = [I_v; I_delta_L; I_delta_R; e_v_previous; e_delta_L_previous; e_delta_R_previous]

xc = zeros(6,1);


%% ========================================================================
%  Storage
%  ========================================================================

X = zeros(12,N);

% Applied actuator inputs
U = zeros(6,N-1);

% Requested total wheel torque
Tau_requested = zeros(1,N-1);

% Applied torque on one wheel
Tau_applied = zeros(1,N-1);

% Store initial state
X(:,1) = x;


%% ========================================================================
%  Simulation
%  ========================================================================

for k = 1:N-1

    % ------------------------------------------------------------
    % Step reference: 0 -> 1 m/s at t = 1 s
    % ------------------------------------------------------------

    if t(k) >= 1
        v_d = 1;
    else
        v_d = 0;
    end
    
    if t(k) >= 2
        delta_d = 0.1;
    else
        delta_d = 0;
    end

    u_c = [v_d; delta_d];


    % ------------------------------------------------------------
    % Controller
    % ------------------------------------------------------------

    [u, xc, tau_total] = limo.controller( ...
        x, ...
        u_c, ...
        xc, ...
        p, ...
        c);


    % ------------------------------------------------------------
    % Vehicle dynamics
    % ------------------------------------------------------------

    xdot = limo.dynamics( ...
        x, ...
        u, ...
        p);


    % ------------------------------------------------------------
    % Euler integration
    % ------------------------------------------------------------

    x = x + Ts*xdot;


    % ------------------------------------------------------------
    % Store data
    % ------------------------------------------------------------

    X(:,k+1) = x;
    U(:,k) = u;

    Tau_requested(k) = tau_total;
    Tau_applied(k) = u(1);

end


%% ========================================================================
%  Extract states
%  ========================================================================

X_position = X(1,:);
Y_position = X(2,:);
psi = X(3,:);

v = X(4,:);

omega_FL = X(5,:);
omega_FR = X(6,:);
omega_RL = X(7,:);
omega_RR = X(8,:);

delta_L = X(9,:);
delta_R = X(10,:);

delta_dot_L = X(11,:);
delta_dot_R = X(12,:);


%% ========================================================================
%  Plot trajectory
%  ========================================================================

figure;

plot(X_position, Y_position, 'LineWidth', 1.5);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');

title('LIMO trajectory');


%% ========================================================================
%  Plot velocity
%  ========================================================================

figure;

plot(t, v, 'LineWidth', 1.5);
hold on;

yline(1, '--');
xline(1, ':');

grid on;

xlabel('Time [s]');
ylabel('Velocity [m/s]');

legend( ...
    'Vehicle velocity', ...
    'Reference', ...
    'Step time', ...
    'Location', 'best');

title('Longitudinal velocity');


%% ========================================================================
%  Plot steering angles
%  ========================================================================

figure;

plot(t, delta_L, 'LineWidth', 1.5);
hold on;

plot(t, delta_R, 'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Steering angle [rad]');

legend( ...
    'Left', ...
    'Right', ...
    'Location', 'best');

title('Front steering angles');


%% ========================================================================
%  Plot wheel angular velocities
%  ========================================================================

figure;

plot(t, omega_FL, 'LineWidth', 1.5);
hold on;

plot(t, omega_FR, 'LineWidth', 1.5);
plot(t, omega_RL, 'LineWidth', 1.5);
plot(t, omega_RR, 'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');

legend( ...
    'FL', ...
    'FR', ...
    'RL', ...
    'RR', ...
    'Location', 'best');

title('Wheel angular velocities');


%% ========================================================================
%  Plot wheel torque
%  ========================================================================

figure;

plot(t(1:end-1), Tau_requested, 'LineWidth', 1.5);
hold on;

plot(t(1:end-1), Tau_applied, 'LineWidth', 1.5);

yline(p.wheel.torque_max, '--');
yline(-p.wheel.torque_max, '--');

grid on;

xlabel('Time [s]');
ylabel('Wheel torque [N.m]');

legend( ...
    'Requested total torque / 4', ...
    'Applied torque', ...
    'Positive saturation', ...
    'Negative saturation', ...
    'Location', 'best');

title('Wheel torque');


%% ========================================================================
%  Plot steering torque
%  ========================================================================

figure;

plot(t(1:end-1), U(5,:), 'LineWidth', 1.5);
hold on;

plot(t(1:end-1), U(6,:), 'LineWidth', 1.5);

yline(p.steering.torque_max, '--');
yline(-p.steering.torque_max, '--');

grid on;

xlabel('Time [s]');
ylabel('Steering torque [N.m]');

legend( ...
    'Left', ...
    'Right', ...
    'Positive saturation', ...
    'Negative saturation', ...
    'Location', 'best');

title('Steering torque');