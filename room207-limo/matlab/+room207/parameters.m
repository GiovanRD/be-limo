function d = parameters()
%PARAMETERS Settings of the Room 207 MATLAB/Simulink demonstrations.
%
%   d = room207.parameters()
%
%   The values follow the ROS 2 demonstration (room207_simu_gui.py and
%   room207_limo.launch.py).

%% ========================================================================
%  Teleoperation (teleoperation/teleop_widget.py)
%  ========================================================================

% Command when Z/S are held
d.teleop.max_speed = 0.5;                               % [m/s]

% Steering angle when Q/D are held
d.teleop.max_steering = pi/6;                           % [rad]

% Command period
d.teleop.sample_time = 0.05;                            % [s]


%% ========================================================================
%  QCar track map (qcar_large_track)
%  ========================================================================

% Physical size of the map, centred on the world origin
d.map.width = 4.88;                                     % [m]
d.map.height = 6.20;                                    % [m]

% Map image, looked for in the environment repository next to
% room207-limo. The view uses a grey background if it is missing.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
d.map.file = fullfile(fileparts(root), 'environment', 'qcar_large_track', ...
    'models', 'qcar_large_track', 'materials', 'textures', 'SDCS_MapLayout.png');


%% ========================================================================
%  View
%  ========================================================================

% Refresh period of the view
d.view.sample_time = 0.05;                              % [s]

% LIMO footprint drawn in the view (length x width)
d.view.footprint = [0.32, 0.22];                        % [m]


%% ========================================================================
%  ROS 2 (connection to the Gazebo demonstration)
%  ========================================================================

% Topic read by the Gazebo Ackermann plugin: [v; gamma], with the sign of
% gamma flipped in reverse (see limo.gazebo_steering_command)
d.ros.command_topic = '/cmd_vel_gazebo';

% Ground-truth odometry published by the plugin
d.ros.odom_topic = '/odom';

end
