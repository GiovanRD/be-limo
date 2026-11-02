function tau_contact = contact_torque(F_x, p)
%CONTACT_TORQUE Compute wheel contact torques.
%
%   tau_contact = limo.contact_torque(F_x, p)
%
%   Input:
%       F_x - Longitudinal contact forces in the wheel frames [N]
%             [FL; FR; RL; RR]
%
%   Output:
%       tau_contact - Contact torques acting on the wheels [N.m]
%                     [FL; FR; RL; RR]
%
%   Model:
%
%       tau_contact,i = R_w * F_x,i
%
%   Sign convention:
%       F_x > 0 corresponds to a force exerted by the ground on the
%       wheel in the forward direction.
%
%       omega > 0 corresponds to wheel rotation in the forward rolling
%       direction.
%
%   Consequently, the wheel rotational dynamics are:
%
%       J_i * omega_dot_i = tau_i - tau_contact,i
%
%   where tau_i is the motor torque.

%% Input validation

if numel(F_x) ~= 4
    error('F_x must contain 4 elements: [FL; FR; RL; RR].');
end

F_x = F_x(:);


%% Parameters

R_w = p.geometry.wheel_radius;


%% Contact torque

tau_contact = R_w * F_x;

end