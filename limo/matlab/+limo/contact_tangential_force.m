function [F_x, F_y] = contact_tangential_force(kappa, alpha, F_n, p)
%CONTACT_TANGENTIAL_FORCE Compute tangential contact forces.
%
%   [F_x, F_y] = limo.contact_tangential_force(kappa, alpha, F_n, p)
%
%   Inputs:
%       kappa - Longitudinal slip ratios [FL; FR; RL; RR]
%       alpha - Wheel drift angles [rad] [FL; FR; RL; RR]
%       F_n   - Normal contact forces [N] [FL; FR; RL; RR]
%
%   Outputs:
%       F_x - Longitudinal tangential forces in wheel frames [N]
%       F_y - Lateral tangential forces in wheel frames [N]
%
%   Model:
%
%       s_i = [kappa_i; tan(alpha_i)]
%
%       F_x,i =  mu_i F_n,i
%                * kappa_i / sqrt(kappa_i^2 + tan(alpha_i)^2 + eps^2)
%
%       F_y,i = -mu_i F_n,i
%                * tan(alpha_i) / sqrt(kappa_i^2 + tan(alpha_i)^2 + eps^2)
%
%   The model is a simplified friction model satisfying
%
%       sqrt(F_x,i^2 + F_y,i^2) <= mu_i F_n,i.
%
%   It is not a detailed tire model.

%% Input validation

if numel(kappa) ~= 4
    error('kappa must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(alpha) ~= 4
    error('alpha must contain 4 elements: [FL; FR; RL; RR].');
end

if numel(F_n) ~= 4
    error('F_n must contain 4 elements: [FL; FR; RL; RR].');
end

kappa = kappa(:);
alpha = alpha(:);
F_n = F_n(:);


%% Parameters

mu = p.contact.friction.mu;
epsilon = p.numerical.slip_epsilon;

% The current model uses the same friction coefficient for all wheels.
if isscalar(mu)
    mu = mu*ones(4,1);
elseif numel(mu) == 4
    mu = mu(:);
else
    error('p.contact.friction.mu must be a scalar or a 4-element vector.');
end


%% Slip measure

s_x = kappa;
s_y = tan(alpha);

s_norm = sqrt(s_x.^2 + s_y.^2 + epsilon^2);


%% Tangential forces

F_x = ...
    mu .* F_n .* s_x ./ s_norm;

F_y = ...
    -mu .* F_n .* s_y ./ s_norm;

end