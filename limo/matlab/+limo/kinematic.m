function xdot = kinematic(x, u, p)
%KINEMATIC Kinematic model of the AgileX LIMO.
%
%   xdot = limo.kinematic(x, u, p)
%
%   State:
%       x = [X; Y; psi]
%
%   Input:
%       u = [v; delta]
%
%   where:
%       X     - longitudinal position in the inertial frame [m]
%       Y     - lateral position in the inertial frame [m]
%       psi   - yaw angle [rad]
%       v     - longitudinal vehicle velocity [m/s]
%       delta - steering angle [rad]
%
%   Model:
%
%       X_dot   = v*cos(psi)
%       Y_dot   = v*sin(psi)
%       psi_dot = v/L*tan(delta)
%
%   The steering angle is saturated according to the physical constraint
%   defined in limo.parameters().

%% Input validation

if numel(x) ~= 3
    error('State x must contain 3 elements: [X; Y; psi].');
end

if numel(u) ~= 2
    error('Input u must contain 2 elements: [v; delta].');
end

x = x(:);
u = u(:);


%% Parameters

L = p.geometry.wheelbase;
delta_max = p.steering.angle_max;


%% State

psi = x(3);


%% Input

v = u(1);
delta = u(2);


%% Steering saturation

delta = min(max(delta, -delta_max), delta_max);


%% Kinematic model

xdot = [
    v*cos(psi)
    v*sin(psi)
    v/L*tan(delta)
];

end