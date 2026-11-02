function F_n = contact_normal_force(xi, xi_dot, p)
%CONTACT_NORMAL_FORCE Compute normal contact forces at the four wheels.
%
%   F_n = limo.contact_normal_force(xi, xi_dot, p)
%
%   Inputs:
%       xi     - Wheel-ground penetrations [m]
%                [FL; FR; RL; RR]
%
%       xi_dot - Penetration rates [m/s]
%                [FL; FR; RL; RR]
%
%   Output:
%       F_n    - Normal contact forces [N]
%                [FL; FR; RL; RR]
%
%   Model:
%
%       F_n,i = k_n * xi_i + c_n * xi_dot_i
%
%   with the unilateral contact constraint:
%
%       F_n,i >= 0.
%
%   The stiffness and damping coefficients are defined in
%   limo.parameters().

%% Input validation

if numel(xi) ~= 4
    error('xi must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(xi_dot) ~= 4
    error('xi_dot must contain 4 elements: [FL; FR; RL; RR].');
end

xi = xi(:);
xi_dot = xi_dot(:);


%% Parameters

k_n = p.contact.normal.stiffness;
c_n = p.contact.normal.damping;


%% Normal contact force

F_n = k_n*xi + c_n*xi_dot;


%% Unilateral contact constraint

F_n = max(F_n, 0);

end