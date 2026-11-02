function omega_dot = wheel_dynamics(omega, tau, tau_contact, p)
%WHEEL_DYNAMICS Rotational dynamics of the four LIMO wheels.
%
%   omega_dot = limo.wheel_dynamics(omega, tau, tau_contact, p)
%
%   Inputs:
%       omega       - Wheel angular velocities [rad/s]
%                     [FL; FR; RL; RR]
%
%       tau         - Motor torques [N.m]
%                     [FL; FR; RL; RR]
%
%       tau_contact - Contact torques opposing the wheel rotation [N.m]
%                     [FL; FR; RL; RR]
%
%   Output:
%       omega_dot   - Wheel angular accelerations [rad/s^2]
%                     [FL; FR; RL; RR]
%
%   Model:
%
%       J_i * omega_dot_i = tau_i - tau_contact_i
%
%   with:
%
%       J_FL = J_FR = J_front
%       J_RL = J_RR = J_rear
%
%   The wheel angular velocity is defined positive in the forward
%   rolling direction. This convention is independent of the sign
%   convention of the URDF wheel joint axes.

%% Input validation

if numel(omega) ~= 4
    error('omega must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(tau) ~= 4
    error('tau must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(tau_contact) ~= 4
    error('tau_contact must contain 4 elements: [FL; FR; RL; RR].');
end

omega = omega(:);
tau = tau(:);
tau_contact = tau_contact(:);


%% Wheel rotational inertias

J = [
    p.wheel.inertia_rotation.front
    p.wheel.inertia_rotation.front
    p.wheel.inertia_rotation.rear
    p.wheel.inertia_rotation.rear
];


%% Wheel dynamics

omega_dot = (tau - tau_contact) ./ J;

end