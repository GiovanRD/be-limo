function xdot = longitudinal_dynamics(x, tau, p)
%LONGITUDINAL_DYNAMICS Simplified longitudinal dynamics of the LIMO.
%
%   xdot = limo.longitudinal_dynamics(x, tau, p)
%
%   State:
%       x = [v; omega_FL; omega_FR; omega_RL; omega_RR]
%
%   Input:
%       tau = [tau_FL; tau_FR; tau_RL; tau_RR] [N.m]
%
%   where:
%       v       - Vehicle longitudinal velocity [m/s]
%       omega_i - Wheel angular velocities [rad/s]
%       tau_i   - Motor torques [N.m]
%
%   Model:
%
%       m * v_dot = sum(F_x,i)
%
%       J_i * omega_dot_i =
%           tau_i - R_w * F_x,i
%
%   The longitudinal contact forces are obtained from the wheel slip
%   ratios through the simplified contact model implemented in
%   limo.contact_tangential_force().
%
%   Assumptions:
%       - Straight-line motion.
%       - Zero steering angle.
%       - No lateral velocity.
%       - Static normal load distribution:
%
%             F_n,i = m*g/4.
%
%       - No aerodynamic drag or rolling resistance.
%
%   This is a simplified longitudinal model. The normal load distribution
%   and full vehicle dynamics will be introduced in higher-fidelity models.

%% Input validation

if numel(x) ~= 5
    error(['State x must contain 5 elements: ' ...
           '[v; omega_FL; omega_FR; omega_RL; omega_RR].']);
end

if numel(tau) ~= 4
    error(['tau must contain 4 elements: ' ...
           '[tau_FL; tau_FR; tau_RL; tau_RR].']);
end

x = x(:);
tau = tau(:);


%% Parameters

m = p.mass.total;
g = p.environment.gravity;

%% State

v = x(1);
omega = x(2:5);


%% Wheel peripheral velocities

v_w = limo.wheel_kinematic(omega, p);


%% Longitudinal contact-point velocities

v_x = v * ones(4,1);
v_y = zeros(4,1);


%% Slip kinematics

[kappa, alpha] = limo.slip_kinematics(v_w, v_x, v_y, p);


%% Static normal loads

F_n = (m*g/4) * ones(4,1);


%% Tangential contact forces

[F_x, ~] = limo.contact_tangential_force(kappa, alpha, F_n, p);


%% Contact torques

tau_contact = limo.contact_torque(F_x, p);


%% Vehicle longitudinal dynamics

v_dot = sum(F_x) / m;


%% Wheel rotational dynamics

omega_dot = limo.wheel_dynamics(omega, tau, tau_contact, p);


%% State derivative

xdot = [
    v_dot
    omega_dot
];

end
