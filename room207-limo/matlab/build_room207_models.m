%BUILD_ROOM207_MODELS Generate the Room 207 Simulink demonstrations.
%
%   Run this script to (re)create, next to it:
%
%       room207_demo.slx    Simulink LIMO in Room 207, no ROS needed
%       room207_gazebo.slx  Same, connected through ROS 2 to the Gazebo
%                           demonstration; the Simulink LIMO runs as a
%                           twin of the Gazebo LIMO
%
%   Both models use the "LIMO (Gazebo)" block of limo_lib.slx and the
%   room207 package (view and keyboard teleoperation).

folder = fileparts(mfilename('fullpath'));
addpath(folder);
room207_setup();

d = room207.parameters();

load_system('limo_lib');

models = {'room207_demo', 'room207_gazebo'};

for i = 1:numel(models)
    if bdIsLoaded(models{i})
        close_system(models{i}, 0);
    end

    if isfile(fullfile(folder, [models{i} '.slx']))
        delete(fullfile(folder, [models{i} '.slx']));
    end
end


%% ========================================================================
%  room207_demo: Simulink LIMO in Room 207
%  ========================================================================

model = 'room207_demo';
new_model(model, {'Simulink LIMO'});

add_command_source(model, d);
add_limo(model, [430 160 710 360]);

add_line(model, 'Command source/1', 'LIMO/1', 'autorouting', 'on');

add_matlab_function([model '/Room 207 view'], [820 80 960 140], { ...
    'function view(t, odom)'
    '%VIEW Draw the LIMO on the Room 207 track (room207.viewer).'
    ''
    'coder.extrinsic(''room207.viewer'');'
    ''
    'room207.viewer(''draw'', t, odom(1:3));'
    }, num2str(d.view.sample_time), {});

add_block('simulink/Sources/Clock', [model '/Clock view'], ...
    'Position', [740 70 770 100]);
add_line(model, 'Clock view/1', 'Room 207 view/1', 'autorouting', 'on');
add_line(model, 'LIMO/1', 'Room 207 view/2', 'autorouting', 'on');

add_to_workspace(model, 'LIMO/1', 'odom');
add_to_workspace(model, 'LIMO/4', 'state');
terminate_ports(model, 'LIMO', [2 3 5 6]);

save_model(model, folder);


%% ========================================================================
%  room207_gazebo: connected to the Gazebo demonstration through ROS 2
%  ========================================================================

model = 'room207_gazebo';
new_model(model, {'Simulink LIMO (twin)', 'Gazebo LIMO (/odom)'});

add_command_source(model, d);
add_limo(model, [430 160 710 360]);

add_line(model, 'Command source/1', 'LIMO/1', 'autorouting', 'on');

% ---- Commands to Gazebo ----------------------------------------------
to_gazebo = [model '/To Gazebo'];
add_subsystem(to_gazebo, [430 40 600 100]);

add_block('simulink/Sources/In1', [to_gazebo '/u'], 'Position', [30 103 60 117]);

add_matlab_function([to_gazebo '/Steering command'], [100 85 240 135], { ...
    'function [linear_x, angular_z] = steering(u)'
    '%STEERING [v; gamma] -> /cmd_vel_gazebo (limo.gazebo_steering_command).'
    ''
    'cmd = limo.gazebo_steering_command(u(1), u(2));'
    'linear_x = cmd(1);'
    'angular_z = cmd(2);'
    }, num2str(d.teleop.sample_time), {});

add_block('ros2lib/Blank Message', [to_gazebo '/Twist'], ...
    'messageClass', 'Message', ...
    'entityType', 'geometry_msgs/Twist', ...
    'messageType', 'geometry_msgs/Twist', ...
    'SampleTime', num2str(d.teleop.sample_time), ...
    'Position', [100 20 220 60]);

add_block('simulink/Signal Routing/Bus Assignment', [to_gazebo '/Bus Assignment'], ...
    'AssignedSignals', 'linear.x,angular.z', ...
    'Position', [300 30 320 140]);

add_block('ros2lib/Publish', [to_gazebo '/Publish'], ...
    'topicSource', 'Specify your own', ...
    'topic', d.ros.command_topic, ...
    'messageType', 'geometry_msgs/Twist', ...
    'Position', [380 65 480 105]);

add_line(to_gazebo, 'u/1', 'Steering command/1', 'autorouting', 'on');
add_line(to_gazebo, 'Twist/1', 'Bus Assignment/1', 'autorouting', 'on');
add_line(to_gazebo, 'Steering command/1', 'Bus Assignment/2', 'autorouting', 'on');
add_line(to_gazebo, 'Steering command/2', 'Bus Assignment/3', 'autorouting', 'on');
add_line(to_gazebo, 'Bus Assignment/1', 'Publish/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(to_gazebo);

add_line(model, 'Command source/1', 'To Gazebo/1', 'autorouting', 'on');

% ---- Odometry from Gazebo --------------------------------------------
from_gazebo = [model '/From Gazebo'];
add_subsystem(from_gazebo, [430 420 600 480]);

add_block('ros2lib/Subscribe', [from_gazebo '/Subscribe'], ...
    'topicSource', 'Specify your own', ...
    'topic', d.ros.odom_topic, ...
    'messageType', 'nav_msgs/Odometry', ...
    'sampleTime', num2str(d.view.sample_time), ...
    'Position', [30 40 130 90]);

add_block('simulink/Signal Routing/Bus Selector', [from_gazebo '/Bus Selector'], ...
    'OutputSignals', strjoin({ ...
        'pose.pose.position.x', 'pose.pose.position.y', ...
        'pose.pose.orientation.x', 'pose.pose.orientation.y', ...
        'pose.pose.orientation.z', 'pose.pose.orientation.w'}, ','), ...
    'Position', [200 40 210 160]);

add_matlab_function([from_gazebo '/Pose'], [280 40 420 160], { ...
    'function pose = odom_pose(is_new, x, y, qx, qy, qz, qw)'
    '%ODOM_POSE Last [X; Y; psi] received on /odom (NaN before the first).'
    ''
    'persistent last'
    ''
    'if isempty(last)'
    '    last = nan(3, 1);'
    'end'
    ''
    'if is_new'
    '    psi = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));'
    '    last = [x; y; psi];'
    'end'
    ''
    'pose = last;'
    }, '-1', {});

add_block('simulink/Sinks/Out1', [from_gazebo '/pose'], 'Position', [480 93 510 107]);

add_line(from_gazebo, 'Subscribe/1', 'Pose/1', 'autorouting', 'on');
add_line(from_gazebo, 'Subscribe/2', 'Bus Selector/1', 'autorouting', 'on');

for k = 1:6
    add_line(from_gazebo, sprintf('Bus Selector/%d', k), sprintf('Pose/%d', k+1), ...
        'autorouting', 'on');
end

add_line(from_gazebo, 'Pose/1', 'pose/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(from_gazebo);

% ---- View --------------------------------------------------------------
add_matlab_function([model '/Room 207 view'], [820 30 960 110], { ...
    'function view(t, odom, gazebo_pose)'
    '%VIEW Draw the Simulink twin and the Gazebo LIMO (room207.viewer).'
    ''
    'coder.extrinsic(''room207.viewer'');'
    ''
    'room207.viewer(''draw'', t, [odom(1:3), gazebo_pose]);'
    }, num2str(d.view.sample_time), {});

add_block('simulink/Sources/Clock', [model '/Clock view'], ...
    'Position', [740 20 770 50]);
add_line(model, 'Clock view/1', 'Room 207 view/1', 'autorouting', 'on');
add_line(model, 'LIMO/1', 'Room 207 view/2', 'autorouting', 'on');
add_line(model, 'From Gazebo/1', 'Room 207 view/3', 'autorouting', 'on');

add_to_workspace(model, 'LIMO/1', 'odom');
add_to_workspace(model, 'From Gazebo/1', 'gazebo_pose');
terminate_ports(model, 'LIMO', [2 3 4 5 6]);

save_model(model, folder);

close_system('limo_lib', 0);


%% ========================================================================
%  Local functions
%  ========================================================================

function new_model(model, names)
%NEW_MODEL Create a model paced in real time with the view callbacks.

new_system(model);

set_param(model, ...
    'Solver', 'ode15s', ...
    'StopTime', 'inf', ...
    'RelTol', '1e-6', ...
    'AbsTol', '1e-8', ...
    'EnablePacing', 'on', ...
    'PacingRate', '1', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'off', ...
    'SaveOutput', 'off', ...
    'SaveFormat', 'Dataset');

set_param(model, 'PreLoadFcn', ...
    'if exist(''room207_setup'', ''file''), room207_setup(); end');

% Names of the robots drawn by the view, e.g. {'Simulink LIMO'}
names_code = ['{' strjoin(strcat('''', names, ''''), ', ') '}'];
set_param(model, 'StartFcn', ...
    ['room207.viewer(''start'', bdroot, ' names_code ');']);
set_param(model, 'StopFcn', 'room207.viewer(''stop'');');

end


function add_command_source(model, d)
%ADD_COMMAND_SOURCE Keyboard or scenario, selected by a manual switch.

source = [model '/Command source'];
add_subsystem(source, [200 160 340 220]);

add_matlab_function([source '/Keyboard'], [120 30 260 80], { ...
    'function u = keyboard_command()'
    '%KEYBOARD_COMMAND [v; gamma] from the keyboard (room207.viewer).'
    '%   Z/S: forward/backward, Q/D: left/right, Space: stop, Esc: end.'
    ''
    'coder.extrinsic(''room207.viewer'');'
    ''
    'u = zeros(2, 1);'
    'u = room207.viewer(''command'');'
    }, num2str(d.teleop.sample_time), {});

add_block('simulink/Sources/Clock', [source '/Clock'], 'Position', [40 125 70 155]);

add_matlab_function([source '/Scenario'], [120 110 260 170], { ...
    'function u = scenario(t)'
    '%SCENARIO Speed and steering angle as a function of time.'
    '%   u = [v (m/s); gamma (rad)], gamma > 0 to the left'
    '%'
    '%   From the Room 207 spawn: leave the parking bay, turn left into the'
    '%   car park, stop, then reverse along the same arc back into the bay.'
    ''
    'v = 0;'
    'gamma = 0;'
    ''
    'if t >= 1 && t < 7.3'
    '    v = 0.3;'
    'elseif t >= 8 && t < 14.3'
    '    v = -0.3;'
    'end'
    ''
    'if (t >= 4.3 && t < 7.3) || (t >= 8 && t < 11)'
    '    gamma = 0.35;'
    'end'
    ''
    'u = [v; gamma];'
    }, '-1', {});

% Double-click the switch to choose the command source
add_block('simulink/Signal Routing/Manual Switch', [source '/Keyboard or scenario'], ...
    'Position', [330 75 360 125]);

add_block('simulink/Sinks/Out1', [source '/u'], 'Position', [420 93 450 107]);

add_line(source, 'Keyboard/1', 'Keyboard or scenario/1', 'autorouting', 'on');
add_line(source, 'Clock/1', 'Scenario/1', 'autorouting', 'on');
add_line(source, 'Scenario/1', 'Keyboard or scenario/2', 'autorouting', 'on');
add_line(source, 'Keyboard or scenario/1', 'u/1', 'autorouting', 'on');

end


function add_limo(model, position)
%ADD_LIMO Link to the LIMO block, driven by [v; gamma] from the Room 207 spawn.

add_block('limo_lib/LIMO (Gazebo)', [model '/LIMO'], ...
    'input_mode', 'Steering angle: [v; gamma]', ...
    'pose_mode', 'Room 207 spawn', ...
    'Position', position);

end


function add_to_workspace(model, source, name)
%ADD_TO_WORKSPACE Log a signal as a timeseries, level with its source port.

parts = split(source, '/');
handles = get_param([model '/' parts{1}], 'PortHandles');
xy = get_param(handles.Outport(str2double(parts{2})), 'Position');

add_block('simulink/Sinks/To Workspace', [model '/' name], ...
    'VariableName', name, ...
    'SaveFormat', 'Timeseries', ...
    'Position', [xy(1)+110, xy(2)-15, xy(1)+190, xy(2)+15]);
add_line(model, source, [name '/1'], 'autorouting', 'on');

end


function terminate_ports(model, block, ports)
%TERMINATE_PORTS Terminate the unused outputs of a block.

handles = get_param([model '/' block], 'PortHandles');

for k = ports
    % Next to the output port
    xy = get_param(handles.Outport(k), 'Position');
    name = sprintf('%s out%d', block, k);
    add_block('simulink/Sinks/Terminator', [model '/' name], ...
        'Position', [xy(1)+40, xy(2)-8, xy(1)+56, xy(2)+8], ...
        'ShowName', 'off');
    add_line(model, sprintf('%s/%d', block, k), [name '/1']);
end

end


function save_model(model, folder)
%SAVE_MODEL Arrange, save and close a model.

Simulink.BlockDiagram.arrangeSystem([model '/Command source']);
save_system(model, fullfile(folder, [model '.slx']));
close_system(model, 0);

fprintf('Saved %s\n', fullfile(folder, [model '.slx']));

end


function add_subsystem(path, position)
%ADD_SUBSYSTEM Add an empty subsystem.

add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

end


function add_matlab_function(path, position, code, sample_time, parameters)
%ADD_MATLAB_FUNCTION Add a MATLAB Function block with the given code.
%
%   parameters lists the function arguments bound to workspace variables
%   (nontunable parameters) instead of input ports.

add_block('simulink/User-Defined Functions/MATLAB Function', path, ...
    'Position', position);

chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = strjoin(code, newline);
set_param(path, 'SystemSampleTime', sample_time);

for i = 1:numel(parameters)
    data = find(chart, '-isa', 'Stateflow.Data', 'Name', parameters{i});
    data.Scope = 'Parameter';
    data.Tunable = false;
end

end
