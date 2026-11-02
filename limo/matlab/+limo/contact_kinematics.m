function [v_x, v_y] = contact_kinematics(v_b, omega_b, delta, p)
%CONTACT_KINEMATICS Velocity at the four LIMO wheel contact points.
%
%   [v_x, v_y] = limo.contact_kinematics(v_b, omega_b, delta, p)
%
%   Inputs:
%       v_b     - Chassis linear velocity in the body frame [m/s]
%                 v_b = [v_x; v_y; v_z]
%
%       omega_b - Chassis angular velocity in the body frame [rad/s]
%                 omega_b = [omega_x; omega_y; omega_z]
%
%       delta   - Front steering angles [rad]
%                 delta = [delta_L; delta_R]
%
%       p       - Physical parameters from limo.parameters()
%
%   Outputs:
%       v_x - Longitudinal velocity at each wheel contact point [m/s]
%             [FL; FR; RL; RR]
%
%       v_y - Lateral velocity at each wheel contact point [m/s]
%             [FL; FR; RL; RR]
%
%   Coordinate convention:
%       x : forward
%       y : left
%       z : upward
%
%   Wheel positions relative to the chassis reference point:
%
%       FL : [ +L/2 ; +T/2 ; 0 ]
%       FR : [ +L/2 ; -T/2 ; 0 ]
%       RL : [ -L/2 ; +T/2 ; 0 ]
%       RR : [ -L/2 ; -T/2 ; 0 ]
%
%   The front wheel frames are rotated by their respective steering
%   angles. The rear wheel frames are aligned with the chassis.

%% Input validation

if numel(v_b) ~= 3
    error('v_b must contain 3 elements: [vx; vy; vz].');
end

if numel(omega_b) ~= 3
    error('omega_b must contain 3 elements: [wx; wy; wz].');
end

if numel(delta) ~= 2
    error('delta must contain 2 elements: [delta_L; delta_R].');
end

v_b = v_b(:);
omega_b = omega_b(:);
delta = delta(:);


%% Parameters

L = p.geometry.wheelbase;
T = p.geometry.track;


%% Wheel positions in the chassis frame

r = [
     L/2,  T/2, 0;    % FL
     L/2, -T/2, 0;    % FR
    -L/2,  T/2, 0;    % RL
    -L/2, -T/2, 0     % RR
];


%% Velocity at each contact point

v_contact = zeros(3, 4);

for i = 1:4
    v_contact(:,i) = v_b + cross(omega_b, r(i,:).');
end


%% Wheel longitudinal/lateral directions

% Front wheels are steered.

e_x = [
    cos(delta(1)),  sin(delta(1)), 0;    % FL
    cos(delta(2)),  sin(delta(2)), 0;    % FR
    1,              0,             0;    % RL
    1,              0,             0     % RR
];

e_y = [
   -sin(delta(1)),  cos(delta(1)), 0;    % FL
   -sin(delta(2)),  cos(delta(2)), 0;    % FR
    0,              1,             0;    % RL
    0,              1,             0     % RR
];


%% Project contact velocities onto wheel frames

v_x = zeros(4,1);
v_y = zeros(4,1);

for i = 1:4
    v_x(i) = e_x(i,:) * v_contact(:,i);
    v_y(i) = e_y(i,:) * v_contact(:,i);
end

end