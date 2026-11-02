function [kappa, alpha] = slip_kinematics(v_w, v_x, v_y, p)
%SLIP_KINEMATICS Compute wheel slip ratio and drift angle.
%
%   [kappa, alpha] = limo.slip_kinematics(v_w, v_x, v_y, p)
%
%   Inputs:
%       v_w - Peripheral wheel velocities [m/s]
%             [FL; FR; RL; RR]
%
%       v_x - Longitudinal contact-point velocities [m/s]
%             [FL; FR; RL; RR]
%
%       v_y - Lateral contact-point velocities [m/s]
%             [FL; FR; RL; RR]
%
%   Outputs:
%       kappa - Longitudinal slip ratios
%               [FL; FR; RL; RR]
%
%       alpha - Wheel drift angles [rad]
%               [FL; FR; RL; RR]
%
%   Models:
%
%       kappa_i =
%           (v_w,i - v_x,i)
%           / max(|v_x,i|, |v_w,i|, epsilon)
%
%       alpha_i =
%           atan2(v_y,i, |v_x,i|)
%
%   The regularization parameter epsilon is defined in
%   limo.parameters().

%% Input validation

if numel(v_w) ~= 4
    error('v_w must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(v_x) ~= 4
    error('v_x must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(v_y) ~= 4
    error('v_y must contain 4 elements: [FL; FR; RL; RR].');
end

v_w = v_w(:);
v_x = v_x(:);
v_y = v_y(:);


%% Parameters

epsilon = p.numerical.slip_epsilon;


%% Longitudinal slip ratio

denominator = max([
    abs(v_x), ...
    abs(v_w), ...
    epsilon * ones(4,1)
], [], 2);

kappa = (v_w - v_x) ./ denominator;


%% Drift angle

alpha = atan2(v_y, abs(v_x));

end