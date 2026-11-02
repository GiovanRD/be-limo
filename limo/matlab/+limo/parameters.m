function p = parameters()
%PARAMETERS Physical parameters of the AgileX LIMO, as simulated in Gazebo.
%
%   p = limo.parameters()
%
%   This structure gathers the physical, geometrical and environmental
%   parameters used by the LIMO models. The values reproduce the Gazebo
%   model spawned by the ENSEM simulations:
%
%       ros2_foxy/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro
%       ros2_foxy/limo_ros2/limo_car/gazebo/ackermann.xacro
%       ros2_foxy/limo_ros2/limo_car/gazebo/sensor.xacro
%
%   together with the gazebo_ros_ackermann_drive plugin (ROS 2 Foxy) and
%   the Gazebo 11 / SDF 1.7 default physics parameters.
%
%   Units are SI unless otherwise stated.
%
%   Frames:
%       base_link : x forward, y left, z up. Origin at the chassis box
%                   centre, 0.145 m above the ground.

%% ========================================================================
%  Geometry
%  ========================================================================

% Wheelbase (distance between the front and rear wheel joints)
p.geometry.wheelbase = 0.24;                          % [m]

% Track width (distance between the left and right wheel joints, i.e.
% between the steering pivots of the front axle)
p.geometry.track = 0.168;                             % [m]

% Wheel radius
p.geometry.wheel_radius = 0.045;                      % [m]

% Wheel diameter
p.geometry.wheel_diameter = ...
    2*p.geometry.wheel_radius;                        % [m]

% Wheel width (length of the collision cylinder)
p.geometry.wheel_width = 0.045;                       % [m]

% Lateral offset of the wheel collision cylinder from its joint.
%
% In the URDF the collision cylinder is shifted by wheel_width/2 towards
% the outside of the robot. The ground contact is therefore 22.5 mm
% outboard of the steering pivot (scrub radius).
p.geometry.contact_offset = ...
    p.geometry.wheel_width/2;                         % [m]

% Track width measured between the contact points
p.geometry.contact_track = ...
    p.geometry.track + 2*p.geometry.contact_offset;   % [m]

% Vertical position of the wheel joints in base_link
p.geometry.wheel_vertical_offset = -0.10;             % [m]

% Height of the base_link origin above the ground
p.geometry.base_height = ...
    p.geometry.wheel_radius ...
    - p.geometry.wheel_vertical_offset;               % [m]

% Chassis collision box (x, y, z) as written in the URDF
p.geometry.chassis_box = [0.19, 0.31, 0.12];          % [m]


%% ========================================================================
%  Mass properties
%  ========================================================================

% Chassis mass (base_link)
p.mass.chassis = 4.34;                                 % [kg]

% Sensor masses, rigidly attached to base_link (lumped by Gazebo)
p.mass.sensors.laser        = 0.1;                     % [kg]
p.mass.sensors.depth_camera = 0.1;                     % [kg]
p.mass.sensors.imu          = 0.01;                    % [kg]

% Steering knuckle (front_*_steer_link) mass
p.mass.steering_link = 0.25;                           % [kg]

% Individual wheel masses
p.mass.wheel.front = 0.5;                              % [kg]
p.mass.wheel.rear  = 0.5;                              % [kg]

% Number of wheels
p.mass.number_of_wheels.front = 2;
p.mass.number_of_wheels.rear  = 2;

% Total vehicle mass
p.mass.total = ...
    p.mass.chassis ...
    + p.mass.sensors.laser ...
    + p.mass.sensors.depth_camera ...
    + p.mass.sensors.imu ...
    + 2*p.mass.steering_link ...
    + p.mass.number_of_wheels.front*p.mass.wheel.front ...
    + p.mass.number_of_wheels.rear*p.mass.wheel.rear; % [kg]


%% ========================================================================
%  Chassis inertia
%  ========================================================================

% Chassis inertia tensor (box_inertial_matrix macro of the URDF).
%
% The macro is called with w = 0.19, l = 0.31, h = 0.12, so that
%   I_xx = m(w^2 + h^2)/12, I_yy = m(h^2 + l^2)/12, I_zz = m(w^2 + l^2)/12.
box = p.geometry.chassis_box;
m_c = p.mass.chassis;

p.inertia.chassis = diag([
    m_c*(box(1)^2 + box(3)^2)/12
    m_c*(box(3)^2 + box(2)^2)/12
    m_c*(box(1)^2 + box(2)^2)/12
]);                                                     % [kg.m^2]


%% ========================================================================
%  Wheel inertia
%  ========================================================================

% Inertia tensor of one wheel in the wheel link frame
% (cylinder_inertial_matrix macro, m = 0.5, r = 0.045, h = 0.045).
%
% The inertial frame is not rotated, so the macro's "axial" value lands
% on the z (vertical) axis, while the spin axis y receives the
% "diametral" value m(3r^2 + h^2)/12.
r_w = p.geometry.wheel_radius;
h_w = p.geometry.wheel_width;

p.inertia.wheel.front = diag([
    p.mass.wheel.front*(3*r_w^2 + h_w^2)/12
    p.mass.wheel.front*(3*r_w^2 + h_w^2)/12
    p.mass.wheel.front*r_w^2/2
]);                                                     % [kg.m^2]

p.inertia.wheel.rear = diag([
    p.mass.wheel.rear*(3*r_w^2 + h_w^2)/12
    p.mass.wheel.rear*(3*r_w^2 + h_w^2)/12
    p.mass.wheel.rear*r_w^2/2
]);                                                     % [kg.m^2]

% Inertia tensor of one steering knuckle (manufacturer data in the URDF)
p.inertia.steering_link = diag([
    0.00525
    0.00035
    0.00525
]);                                                     % [kg.m^2]


%% ========================================================================
%  Composite rigid body
%  ========================================================================

% Point masses and inertias of all links, expressed in base_link.
% Columns: mass, x, y, z, I_zz (about the link's own centre of mass).
L = p.geometry.wheelbase;
T = p.geometry.track;
z_w = p.geometry.wheel_vertical_offset;

links = [
    p.mass.chassis,              0,       0,    0,      p.inertia.chassis(3,3)
    p.mass.sensors.laser,        0.1236,  0,   -0.01,   1e-6
    p.mass.sensors.depth_camera, 0.1,     0,    0.065,  1e-6
    p.mass.sensors.imu,          0,       0,   -0.1,    1e-7
    p.mass.steering_link,        L/2,     T/2,  z_w,    p.inertia.steering_link(3,3)
    p.mass.steering_link,        L/2,    -T/2,  z_w,    p.inertia.steering_link(3,3)
    p.mass.wheel.front,          L/2,     T/2,  z_w,    p.inertia.wheel.front(3,3)
    p.mass.wheel.front,          L/2,    -T/2,  z_w,    p.inertia.wheel.front(3,3)
    p.mass.wheel.rear,          -L/2,     T/2,  z_w,    p.inertia.wheel.rear(3,3)
    p.mass.wheel.rear,          -L/2,    -T/2,  z_w,    p.inertia.wheel.rear(3,3)
];

m_i = links(:,1);
r_i = links(:,2:4);

% Centre of mass of the whole vehicle in base_link
p.com.position = (m_i.'*r_i / sum(m_i)).';              % [m]

% Height of the centre of mass above the ground
p.com.height = ...
    p.geometry.base_height + p.com.position(3);         % [m]

% Yaw inertia of the whole vehicle about its centre of mass
d_i = r_i(:,1:2) - p.com.position(1:2).';
p.inertia.yaw = ...
    sum(links(:,5)) + sum(m_i .* sum(d_i.^2, 2));      % [kg.m^2]


%% ========================================================================
%  Wheel rotational dynamics
%  ========================================================================

% Wheel rotational inertia around the wheel rotation axis.
%
% From the URDF:
%   left wheel axis  = [0  1 0]
%   right wheel axis = [0 -1 0] in a frame rotated by pi about x
%
% Both axes point to +y of base_link, so a positive joint velocity
% always corresponds to forward rolling.

p.wheel.inertia_rotation.front = ...
    p.inertia.wheel.front(2,2);                         % [kg.m^2]

p.wheel.inertia_rotation.rear = ...
    p.inertia.wheel.rear(2,2);                          % [kg.m^2]

% Maximum wheel motor torque.
%
% The wheel joints have no effort limit and the Foxy Ackermann plugin
% does not read <max_wheel_torque>: the drive torque is unbounded.
p.wheel.torque_max = Inf;                               % [N.m]


%% ========================================================================
%  Steering dynamics
%  ========================================================================

% Steering angle limit (joint limit)
p.steering.angle_max = 0.523598767;                     % [rad]

% Steering rate limit (joint velocity limit)
p.steering.rate_max = 3.0;                              % [rad/s]

% Maximum steering actuator torque (joint effort limit)
p.steering.torque_max = 20;                             % [N.m]

% Steering viscous damping
p.steering.damping = 0.1;                               % [N.m.s/rad]

% Steering friction torque
p.steering.friction = 0.1;                              % [N.m]

% Steering joint inertia: knuckle + wheel, about the vertical pivot axis.
p.steering.inertia.front_left = ...
    p.inertia.steering_link(3,3) + p.inertia.wheel.front(3,3);  % [kg.m^2]

p.steering.inertia.front_right = ...
    p.inertia.steering_link(3,3) + p.inertia.wheel.front(3,3);  % [kg.m^2]

% Joint stop model (ODE uses a rigid stop; it is approximated here by a
% stiff spring-damper acting beyond the joint limit).
p.steering.stop.stiffness = 1e4;                        % [N.m/rad]
p.steering.stop.damping   = 5;                          % [N.m.s/rad]


%% ========================================================================
%  Wheel-ground contact
%  ========================================================================

% Normal contact stiffness and damping (ODE <kp>, <kd> defaults).
%
% The contact is practically rigid. The MATLAB model computes the normal
% loads from the static balance and the load transfer instead.
p.contact.normal.stiffness = 1e12;                      % [N/m]
p.contact.normal.damping = 1;                           % [N.s/m]

% Tangential friction coefficient.
%
% Neither the wheels nor the floor define <surface><friction>, so ODE
% uses mu = mu2 = 1. The Ackermann plugin selects the "cone_model"
% friction law: |F_t| <= mu*F_n.
p.contact.friction.mu = 1;

% Regularisation velocity of the friction cone.
%
% ODE enforces exact sticking. The MATLAB model uses
%   F_t = -mu*F_n * s / sqrt(|s|^2 + v_eps^2)
% where s is the sliding velocity of the contact point. Smaller values
% get closer to ODE but make the equations stiffer.
p.contact.friction.regularization_velocity = 1e-3;      % [m/s]


%% ========================================================================
%  Environment
%  ========================================================================

% Gravitational acceleration (SDF world default)
p.environment.gravity = 9.8;                            % [m/s^2]


%% ========================================================================
%  Gazebo simulation and Ackermann drive plugin
%  ========================================================================

% Physics step (SDF default <max_step_size>)
p.gazebo.physics_step = 0.001;                          % [s]

% Plugin update rate (<update_rate>).
%
% Joint::SetForce() only acts during the next physics step (ODE clears
% its force accumulators after each step). Since the plugin runs at
% 100 Hz, the commanded efforts are applied during 1 ms every 10 ms.
p.gazebo.plugin.sample_time = 1/100;                    % [s]

% PID gains [P I D] and integral ranges [min max]
% (gazebo::common::PID, cmd = -(P*e + I*int(e) + D*de/dt), e = actual - target)
p.gazebo.plugin.linear_pid = [0.5, 10, 0.001];
p.gazebo.plugin.linear_i_range = [0, 0];

p.gazebo.plugin.steering_pid = [10000, 100, 10];
p.gazebo.plugin.steering_i_range = [0, 0];

% Command limits (<max_steer>, default <max_speed>)
p.gazebo.plugin.max_steer = 0.523598767;                % [rad]
p.gazebo.plugin.max_speed = 20;                         % [m/s]

% Geometry computed by the plugin from the collision positions at load
% time (the <wheel_separation> and <wheel_diameter> tags are not read).
p.gazebo.plugin.wheel_separation = p.geometry.contact_track;  % [m]
p.gazebo.plugin.wheel_base = p.geometry.wheelbase;            % [m]
p.gazebo.plugin.wheel_radius = p.geometry.wheel_radius;       % [m]

% gazebo_twist_adapter node (limo_car/scripts/gazebo_twist_adapter.py)
p.gazebo.adapter.wheelbase = 0.24;                      % [m]
p.gazebo.adapter.max_steering_angle = atan(0.24/0.55);  % [rad]
p.gazebo.adapter.min_speed = 0.001;                     % [m/s]
p.gazebo.adapter.command_timeout = 0.5;                 % [s]


%% ========================================================================
%  Sensors
%  ========================================================================

% IMU (imu_link, update rate 100 Hz, no noise in the Gazebo model)
p.sensors.imu.sample_time = 1/100;                      % [s]
p.sensors.imu.position = [0; 0; -0.1];                  % [m]

% Odometry published by the plugin (ground truth, 100 Hz)
p.sensors.odom.sample_time = 1/100;                     % [s]


%% ========================================================================
%  Initial conditions
%  ========================================================================

% Initial pose [X; Y; psi] of base_link in the world frame
p.initial.pose = [0; 0; 0];

% Spawn pose of the Room 207 simulation (room207-limo launch file)
p.initial.room207_pose = [-0.725347; -0.086613; 1.5708];


%% ========================================================================
%  Numerical parameters
%  ========================================================================

% Regularization parameter used in the longitudinal slip definition.
%
% kappa_i =
% (R_w*phi_dot_i - v_x_i) /
% max(|v_x_i|, |R_w*phi_dot_i|, epsilon)
p.numerical.slip_epsilon = 1e-6;

% Regularisation velocity of the steering joint Coulomb friction
p.numerical.joint_friction_velocity = 1e-3;             % [rad/s]

% Fixed-point iterations of the load transfer computation
p.numerical.load_transfer_iterations = 3;

end
