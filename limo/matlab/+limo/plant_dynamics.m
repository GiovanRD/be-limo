function [xdot, aux] = plant_dynamics(x, u, p)
%PLANT_DYNAMICS High-fidelity planar dynamics of the LIMO, as in Gazebo.
%
%   [xdot, aux] = limo.plant_dynamics(x, u, p)
%
%   State (14 elements):
%       x = [X; Y; psi;                      base_link pose in the world
%            v_x; v_y; r;                    base_link velocity (body frame)
%            omega_FL; omega_FR;             wheel spin rates [rad/s]
%            omega_RL; omega_RR;
%            delta_L; delta_R;               steering joint angles [rad]
%            delta_dot_L; delta_dot_R]       steering joint rates [rad/s]
%
%   Input (6 elements), joint efforts as applied by Joint::SetForce():
%       u = [tau_FL; tau_FR; tau_RL; tau_RR;  wheel torques [N.m]
%            tau_delta_L; tau_delta_R]        steering torques [N.m]
%
%   Outputs:
%       xdot - State derivative
%       aux  - Structure with the contact forces and loads:
%                F_x, F_y   longitudinal/lateral force in each wheel frame [N]
%                F_n        normal load of each wheel [N]
%                a          acceleration of the centre of mass (body) [m/s^2]
%
%   Model:
%
%       The chassis, sensors, knuckles and wheels form one rigid body
%       moving in the plane. The knuckles rotate about vertical pivots
%       located at the wheel joints, and the wheels spin about their axles.
%
%       Each wheel touches the ground at the centre of its collision
%       cylinder, which is offset by contact_offset outboard of the joint.
%       The ground force follows a regularised Coulomb friction cone:
%
%           F_t = -mu*F_n * s / sqrt(|s|^2 + v_eps^2)
%
%       where s is the sliding velocity of the wheel at the contact point.
%
%       The normal loads come from the static balance plus the
%       longitudinal and lateral load transfer of a rigid chassis
%       (chassis pitch and roll are neglected).
%
%       Equations of motion (body frame, G = centre of mass):
%
%           m (v_x_dot - r v_y - r^2 x_G) = sum F_x
%           m (v_y_dot + r v_x + r_dot x_G) = sum F_y
%           (I_G - 2 J_s) r_dot = M_G - sum (tau_s,j + M_k,j)
%           J_s (r_dot + delta_ddot_j) = tau_s,j + M_k,j
%           J_w omega_dot_i = tau_i - R_w F_x,i
%
%       with M_k,j the moment of the tyre force about the steering pivot
%       and tau_s,j the net steering joint torque (actuator, damping,
%       friction and joint stop).

%% Input validation

if numel(x) ~= 14
    error(['State x must contain 14 elements: ' ...
           '[X; Y; psi; v_x; v_y; r; omega_FL; omega_FR; omega_RL; ' ...
           'omega_RR; delta_L; delta_R; delta_dot_L; delta_dot_R].']);
end

if numel(u) ~= 6
    error(['Input u must contain 6 elements: ' ...
           '[tau_FL; tau_FR; tau_RL; tau_RR; tau_delta_L; tau_delta_R].']);
end

x = x(:);
u = u(:);


%% Parameters

L = p.geometry.wheelbase;
T = p.geometry.track;
e = p.geometry.contact_offset;
T_c = p.geometry.contact_track;
R_w = p.geometry.wheel_radius;

m = p.mass.total;
x_G = p.com.position(1);
h_G = p.com.height;
I_G = p.inertia.yaw;
g = p.environment.gravity;

J_w = [
    p.wheel.inertia_rotation.front
    p.wheel.inertia_rotation.front
    p.wheel.inertia_rotation.rear
    p.wheel.inertia_rotation.rear
];

J_s = [
    p.steering.inertia.front_left
    p.steering.inertia.front_right
];

mu = p.contact.friction.mu;
v_eps = p.contact.friction.regularization_velocity;


%% State

psi = x(3);
v_x = x(4);
v_y = x(5);
r = x(6);
omega = x(7:10);
delta = x(11:12);
delta_dot = x(13:14);


%% Inputs

tau_w = u(1:4);
tau_delta = u(5:6);


%% Contact geometry (body frame, origin below base_link)

% Wheel headings
heading = [delta(1); delta(2); 0; 0];

% Wheel joints (steering pivots for the front wheels)
pivot = [
     L/2,  T/2
     L/2, -T/2
    -L/2,  T/2
    -L/2, -T/2
];

% +1 for the left wheels, -1 for the right wheels
side = [1; -1; 1; -1];

% Wheel frame unit vectors
t_dir = [cos(heading), sin(heading)];
n_dir = [-sin(heading), cos(heading)];

% Contact points: collision centres, offset outboard along the axle
contact = pivot + (side*e) .* n_dir;

% Rate of change of the wheel heading relative to the chassis
heading_dot = [delta_dot(1); delta_dot(2); 0; 0];


%% Sliding velocities

% Velocity of the wheel material point at the contact (rigid motion of
% the chassis + rotation of the knuckle about its pivot)
v_cx = v_x - r*contact(:,2) - heading_dot.*(contact(:,2) - pivot(:,2));
v_cy = v_y + r*contact(:,1) + heading_dot.*(contact(:,1) - pivot(:,1));

% Projection in the wheel frames
v_t = t_dir(:,1).*v_cx + t_dir(:,2).*v_cy;
v_n = n_dir(:,1).*v_cx + n_dir(:,2).*v_cy;

% Sliding velocity (wheel spin removes R*omega along the wheel heading)
s_t = v_t - R_w*omega;
s_n = v_n;

s_norm = sqrt(s_t.^2 + s_n.^2 + v_eps^2);


%% Normal loads and tangential forces (fixed point on the load transfer)

% Static axle loads
F_front_static = m*g*(x_G + L/2)/L;
F_rear_static  = m*g*(L/2 - x_G)/L;

a = zeros(2,1);
F_n = zeros(4,1);
F_t = zeros(4,1);
F_l = zeros(4,1);
F_body = zeros(4,2);

for k = 1:p.numerical.load_transfer_iterations

    % Longitudinal load transfer
    F_front = F_front_static - m*a(1)*h_G/L;
    F_rear  = F_rear_static  + m*a(1)*h_G/L;

    % Lateral load transfer, shared in proportion to the axle loads
    dF_lat = m*a(2)*h_G/T_c;
    share_front = F_front_static/(m*g);
    share_rear  = F_rear_static/(m*g);

    F_n = [
        F_front/2 - share_front*dF_lat
        F_front/2 + share_front*dF_lat
        F_rear/2  - share_rear*dF_lat
        F_rear/2  + share_rear*dF_lat
    ];

    % A wheel cannot pull on the ground
    F_n = max(F_n, 0);

    % Friction cone
    F_t = -mu*F_n.*s_t./s_norm;
    F_l = -mu*F_n.*s_n./s_norm;

    % Forces in the body frame
    F_body = F_t.*t_dir + F_l.*n_dir;

    % Acceleration of the centre of mass
    a = sum(F_body, 1).'/m;
end


%% Resultant forces and moments

F_sum = sum(F_body, 1).';

% Moment of each contact force about the centre of mass
arm_G = contact - [x_G, 0];
M_G = sum(arm_G(:,1).*F_body(:,2) - arm_G(:,2).*F_body(:,1));

% Moment of the front contact forces about their steering pivots
arm_k = contact(1:2,:) - pivot(1:2,:);
M_k = arm_k(:,1).*F_body(1:2,2) - arm_k(:,2).*F_body(1:2,1);


%% Steering joint torques

delta_max = p.steering.angle_max;

% Coulomb friction (regularised) and viscous damping
tau_friction = p.steering.friction ...
    * delta_dot./sqrt(delta_dot.^2 + p.numerical.joint_friction_velocity^2);

tau_damping = p.steering.damping*delta_dot;

% Joint stop
over = max(delta - delta_max, 0) + min(delta + delta_max, 0);
tau_stop = -p.steering.stop.stiffness*over ...
    - p.steering.stop.damping*delta_dot.*(over ~= 0);

% Effort limit
tau_act = min(max(tau_delta, -p.steering.torque_max), p.steering.torque_max);

tau_s = tau_act - tau_damping - tau_friction + tau_stop;


%% Equations of motion

r_dot = (M_G - sum(tau_s + M_k)) / (I_G - sum(J_s));

v_x_dot = F_sum(1)/m + r*v_y + r^2*x_G;
v_y_dot = F_sum(2)/m - r*v_x - r_dot*x_G;

delta_ddot = (tau_s + M_k)./J_s - r_dot;

omega_dot = (tau_w - R_w*F_t)./J_w;


%% Pose kinematics

X_dot = cos(psi)*v_x - sin(psi)*v_y;
Y_dot = sin(psi)*v_x + cos(psi)*v_y;


%% State derivative

xdot = [
    X_dot
    Y_dot
    r
    v_x_dot
    v_y_dot
    r_dot
    omega_dot
    delta_dot
    delta_ddot
];


%% Auxiliary outputs

aux.F_x = F_t;
aux.F_y = F_l;
aux.F_n = F_n;
aux.a = a;

end
