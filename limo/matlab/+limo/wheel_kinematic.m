function v_w = wheel_kinematic(omega, p)
%WHEEL_KINEMATIC Peripheral velocity of the LIMO wheels.
%
%   v_w = limo.wheel_kinematic(omega, p)
%
%   Input:
%       omega = [omega_FL; omega_FR; omega_RL; omega_RR]
%               Wheel angular velocities [rad/s]
%
%   Output:
%       v_w   = [v_FL; v_FR; v_RL; v_RR]
%               Peripheral wheel velocities [m/s]
%
%   Model:
%
%       v_w,i = R_w * omega_i
%
%   The wheel radius is common to all four wheels.

%% Input validation

if numel(omega) ~= 4
    error('omega must contain 4 elements: [FL; FR; RL; RR].');
end

omega = omega(:);

%% Parameters

R_w = p.geometry.wheel_radius;

%% Wheel peripheral velocities

v_w = R_w * omega;

end
