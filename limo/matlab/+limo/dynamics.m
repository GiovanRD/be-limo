function [xdot, sens_data] = dynamics(x, u, p)
%DYNAMICS Dynamic model of the AgileX LIMO with perfect wheel-ground adhesion.
%
%   [xdot, sens_data] = limo.dynamics(x, u, p)
%
%   Returns both the state derivatives and physical sensor-friendly data 
%   (accelerations, angular rates) for IMU simulation.

%% Check dimensions
if numel(x) ~= 12
    error(['State x must contain 12 elements: ' ...
           '[X; Y; psi; v; omega_FL; omega_FR; omega_RL; omega_RR; ' ...
           'delta_L; delta_R; delta_dot_L; delta_dot_R].']);
end

if numel(u) ~= 6
    error(['Input u must contain 6 elements: ' ...
           '[tau_FL; tau_FR; tau_RL; tau_RR; ' ...
           'tau_delta_L; tau_delta_R].']);
end

x = x(:);
u = u(:);

%% State
X = x(1);
Y = x(2);
psi = x(3);
v = x(4);
omega = x(5:8);
delta = x(9:10);
delta_dot = x(11:12);

%% Inputs
tau = u(1:4);
tau_delta = u(5:6);

%% Kinematic vehicle motion
delta_eq = mean(delta);

xdot_kin = limo.kinematic( ...
    [X; Y; psi], ...
    [v; delta_eq], ...
    p);

% Extraction de la vitesse de lacet (psi_dot) depuis la cinématique
psi_dot = xdot_kin(3);

%% Perfect-adhesion longitudinal dynamics
R_w = p.geometry.wheel_radius;
m = p.mass.total;

J = [
    p.wheel.inertia_rotation.front
    p.wheel.inertia_rotation.front
    p.wheel.inertia_rotation.rear
    p.wheel.inertia_rotation.rear
];

% Equivalent translational mass including wheel rotational inertia
m_eq = m + sum(J)/R_w^2;

% Vehicle acceleration (ax)
v_dot = sum(tau) / (R_w * m_eq);

% Perfect rolling constraint
omega_dot = (v_dot / R_w) * ones(4,1);

%% Steering dynamics
tau_friction = ...
    p.steering.friction * sign(delta_dot);

J_delta = [
    p.steering.inertia.front_left
    p.steering.inertia.front_right
];

delta_ddot = ( ...
      tau_delta ...
    - p.steering.damping .* delta_dot ...
    - tau_friction ...
    ) ./ J_delta;

%% State derivative
xdot = [
    xdot_kin
    v_dot
    omega_dot
    delta_dot
    delta_ddot
];

%% Sensor Data Generation (pour l'IMU)
% - Accélération longitudinale (ax)
% - Accélération latérale/centripète (ay = v * psi_dot)
% - Vitesses de rotation (gyroscope)
sens_data.ax   = v_dot;
sens_data.ay   = v * psi_dot; 
sens_data.az   = 0.0;
sens_data.gx   = 0.0;       % Roulement plan (supposé nul)
sens_data.gy   = 0.0;       % Tangage plan (supposé nul)
sens_data.gz   = psi_dot;   % Vitesse de lacet
sens_data.yaw  = psi;       % Cap brut

end