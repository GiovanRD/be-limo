function [u, xc, tau_total] = controller(x, u_c, xc, p, c)
%CONTROLLER PID controller for the AgileX LIMO with Ackermann correction.

%% ========================================================================
% Input validation
% ========================================================================
if numel(x) ~= 12
    error('State x must contain 12 elements.');
end
if numel(u_c) ~= 2
    error('u_c must contain 2 elements: [v_d; delta_d].');
end
if numel(xc) ~= 6
    error(['Controller state xc must contain 6 elements: ' ...
           '[I_v; I_delta_L; I_delta_R; e_v_prev; e_delta_L_prev; e_delta_R_prev].']);
end

x = x(:); u_c = u_c(:); xc = xc(:);

%% ========================================================================
% State & Command
% ========================================================================
v = x(4);
delta_L = x(9);
delta_R = x(10);

v_d = u_c(1);
delta_d = u_c(2);

%% ========================================================================
% Controller state
% ========================================================================
I_v = xc(1);
I_delta_L = xc(2);
I_delta_R = xc(3);
e_v_prev = xc(4);
e_delta_L_prev = xc(5);
e_delta_R_prev = xc(6);

%% ========================================================================
% Ackermann Geometric Correction
% ========================================================================
L = p.geometry.wheelbase;
W = p.geometry.track;

if abs(delta_d) < 1e-4
    % Ligne droite
    delta_d_L = delta_d;
    delta_d_R = delta_d;
else
    % Rayon de courbure au centre de l'essieu arrière
    R = L / tan(delta_d);
    
    % Angles d'Ackermann pour chaque roue
    delta_d_L = atan(L / (R - W/2)); % Roue gauche
    delta_d_R = atan(L / (R + W/2)); % Roue droite
end

%% ========================================================================
% Errors & Derivatives
% ========================================================================
Ts = c.sample_time;

e_v = v_d - v;
e_delta_L = delta_d_L - delta_L;
e_delta_R = delta_d_R - delta_R;

de_v = (e_v - e_v_prev) / Ts;
de_delta_L = (e_delta_L - e_delta_L_prev) / Ts;
de_delta_R = (e_delta_R - e_delta_R_prev) / Ts;

I_v = I_v + Ts*e_v;
I_delta_L = I_delta_L + Ts*e_delta_L;
I_delta_R = I_delta_R + Ts*e_delta_R;

%% ========================================================================
% PIDs
% ========================================================================
% Velocity
tau_total = c.velocity.Kp * e_v + c.velocity.Ki * I_v + c.velocity.Kd * de_v;

% Direction (Indépendante pour chaque roue)
tau_delta_L = c.steering.Kp * e_delta_L + c.steering.Ki * I_delta_L + c.steering.Kd * de_delta_L;
tau_delta_R = c.steering.Kp * e_delta_R + c.steering.Ki * I_delta_R + c.steering.Kd * de_delta_R;

%% ========================================================================
% Actuator saturation & Control allocation
% ========================================================================
% Rear wheel torque allocation
tau_rear = tau_total / 2;
tau_rear = min(max(tau_rear, -p.wheel.torque_max), p.wheel.torque_max);

tau = [0; 0; tau_rear; tau_rear];

% Steering torque saturation
tau_delta_L = min(max(tau_delta_L, -p.steering.torque_max), p.steering.torque_max);
tau_delta_R = min(max(tau_delta_R, -p.steering.torque_max), p.steering.torque_max);

tau_delta_vec = [tau_delta_L; tau_delta_R];

%% ========================================================================
% Output
% ========================================================================
u = [tau; tau_delta_vec];

xc = [I_v; I_delta_L; I_delta_R; e_v; e_delta_L; e_delta_R];

end