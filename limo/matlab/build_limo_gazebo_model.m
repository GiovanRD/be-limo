%BUILD_LIMO_GAZEBO_MODEL Generate the Simulink library and demo model.
%
%   Run this script from any folder to (re)create, next to it:
%
%       limo_lib.slx     Library with the masked "LIMO (Gazebo)" block
%       limo_gazebo.slx  Demo model using a link to that block
%
%   The "LIMO (Gazebo)" block reproduces the LIMO simulated in Gazebo:
%
%       Command        variant chosen in the mask:
%                        Steering angle   [v; gamma] -> Gazebo plugin
%                        cmd_vel          [v; omega_z] -> adapter -> plugin
%                        Joint efforts    torque input
%       LIMO plant     limo.plant_dynamics
%       Sensors        odometry, IMU, joint states
%
%   All the equations live in the +limo package; the blocks only call
%   these functions. Edit the package, not the generated blocks.

folder = fileparts(mfilename('fullpath'));
addpath(folder);
addpath(fullfile(folder, 'images'));

lib = 'limo_lib';
model = 'limo_gazebo';

for name = {model, lib}
    if bdIsLoaded(name{1})
        close_system(name{1}, 0);
    end

    if isfile(fullfile(folder, [name{1} '.slx']))
        delete(fullfile(folder, [name{1} '.slx']));
    end
end


%% ========================================================================
%  Library
%  ========================================================================

new_system(lib, 'Library');

set_param(lib, 'PostLoadFcn', ...
    'addpath(fileparts(get_param(''limo_lib'', ''FileName'')));');

limo_sys = [lib '/LIMO (Gazebo)'];
add_subsystem(limo_sys, [100 100 380 300]);


%% ========================================================================
%  LIMO (Gazebo)
%  ========================================================================

add_block('simulink/Sources/In1', [limo_sys '/u'], ...
    'Position', [30 113 60 127]);

command_sys = [limo_sys '/Command'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', command_sys, ...
    'Position', [290 90 450 150]);
Simulink.SubSystem.deleteContents(command_sys);

plant_sys = [limo_sys '/LIMO plant'];
add_subsystem(plant_sys, [510 90 640 170]);

sensors_sys = [limo_sys '/Sensors'];
add_subsystem(sensors_sys, [710 80 840 200]);

limo_outputs = {'odom', 'imu', 'joints', 'state', 'tau', 'contact'};

for i = 1:numel(limo_outputs)
    y = 60 + 50*i;
    add_block('simulink/Sinks/Out1', [limo_sys '/' limo_outputs{i}], ...
        'Port', num2str(i), 'Position', [920 y 950 y+14]);
end


%% ========================================================================
%  Command (variant subsystem)
%  ========================================================================

add_block('simulink/Sources/In1', [command_sys '/u'], ...
    'Position', [30 53 60 67]);
add_block('simulink/Sources/In1', [command_sys '/x'], ...
    'Port', '2', 'Position', [30 113 60 127]);
add_block('simulink/Sinks/Out1', [command_sys '/tau'], ...
    'Position', [400 83 430 97]);

% Choice 1: speed and steering angle, sent to the Gazebo plugin
steering_sys = [command_sys '/Steering angle'];
add_plugin_choice(steering_sys, [150 20 300 60], 'input_mode == 1', ...
    'Steering command', { ...
    'function cmd_gazebo = steering(v_gamma)'
    '%STEERING [v; gamma] -> /cmd_vel_gazebo (limo.gazebo_steering_command).'
    ''
    'cmd_gazebo = limo.gazebo_steering_command(v_gamma(1), v_gamma(2));'
    }, {});

% Choice 2: /cmd_vel through the twist adapter and the Gazebo plugin
cmd_vel_sys = [command_sys '/cmd_vel'];
add_plugin_choice(cmd_vel_sys, [150 80 300 120], 'input_mode == 2', ...
    'Twist adapter', { ...
    'function cmd_gazebo = adapter(cmd_vel, limo_p)'
    '%ADAPTER /cmd_vel -> /cmd_vel_gazebo (limo.gazebo_twist_adapter).'
    ''
    'cmd_gazebo = limo.gazebo_twist_adapter(cmd_vel(1), cmd_vel(2), limo_p);'
    }, {'limo_p'});

% Choice 3: joint efforts given directly by the user
efforts_sys = [command_sys '/Joint efforts'];
add_choice(efforts_sys, [150 140 300 180], 'input_mode == 3');

add_block('simulink/Sinks/Terminator', [efforts_sys '/Terminator'], ...
    'Position', [150 105 170 125]);

add_line(efforts_sys, 'u/1', 'tau/1', 'autorouting', 'on');
add_line(efforts_sys, 'x/1', 'Terminator/1', 'autorouting', 'on');

set_param(command_sys, ...
    'VariantControlMode', 'expression', ...
    'VariantActivationTime', 'update diagram');


%% ========================================================================
%  LIMO plant
%  ========================================================================

add_block('simulink/Sources/In1', [plant_sys '/tau'], ...
    'Position', [30 93 60 107]);

add_matlab_function([plant_sys '/Dynamics'], [120 75 260 135], { ...
    'function [xdot, contact] = dynamics(tau, x, limo_p)'
    '%DYNAMICS State derivative of the LIMO (limo.plant_dynamics).'
    '%'
    '%   contact = [F_x (4); F_y (4); F_n (4)] in the wheel frames [N],'
    '%   wheel order FL, FR, RL, RR.'
    ''
    '[xdot, aux] = limo.plant_dynamics(x, tau, limo_p);'
    ''
    'contact = [aux.F_x; aux.F_y; aux.F_n];'
    }, '-1', {'limo_p'});

add_block('simulink/Continuous/Integrator', [plant_sys '/Integrator'], ...
    'InitialCondition', 'limo.initial_state(limo_p)', ...
    'Position', [320 82 350 113]);

add_block('simulink/Sinks/Out1', [plant_sys '/x'], ...
    'Port', '1', 'Position', [420 91 450 105]);
add_block('simulink/Sinks/Out1', [plant_sys '/contact'], ...
    'Port', '2', 'Position', [420 161 450 175]);

add_line(plant_sys, 'tau/1', 'Dynamics/1', 'autorouting', 'on');
add_line(plant_sys, 'Dynamics/1', 'Integrator/1', 'autorouting', 'on');
add_line(plant_sys, 'Integrator/1', 'x/1', 'autorouting', 'on');
add_line(plant_sys, 'Integrator/1', 'Dynamics/2', 'autorouting', 'on');
add_line(plant_sys, 'Dynamics/2', 'contact/1', 'autorouting', 'on');


%% ========================================================================
%  Sensors
%  ========================================================================

add_block('simulink/Sources/In1', [sensors_sys '/x'], ...
    'Position', [30 143 60 157]);

add_matlab_function([sensors_sys '/Odometry'], [130 40 250 90], { ...
    'function odom = odometry(x)'
    '%ODOMETRY Ground-truth odometry published by the Ackermann plugin.'
    '%   odom = [X; Y; psi; v_x; v_y; r] of base_link'
    ''
    'odom = x(1:6);'
    }, 'limo_p.sensors.odom.sample_time', {});

add_matlab_function([sensors_sys '/IMU'], [130 125 250 175], { ...
    'function imu = imu_sensor(x, limo_p)'
    '%IMU_SENSOR Gazebo IMU (limo.imu_gazebo).'
    '%   imu = [q_x; q_y; q_z; q_w; omega (3); a (3)]'
    ''
    'persistent v_last'
    ''
    'if isempty(v_last)'
    '    % Seeded with the velocity at load time'
    '    v_last = [cos(x(3))*x(4) - sin(x(3))*x(5); sin(x(3))*x(4) + cos(x(3))*x(5)];'
    'end'
    ''
    '[imu, v_last] = limo.imu_gazebo(x, v_last, limo_p);'
    }, 'limo_p.sensors.imu.sample_time', {'limo_p'});

add_matlab_function([sensors_sys '/Joint states'], [130 210 250 260], { ...
    'function joints = joint_states(x)'
    '%JOINT_STATES Wheel rates and steering angles.'
    '%   joints = [omega_FL; omega_FR; omega_RL; omega_RR; delta_L; delta_R]'
    ''
    'joints = x(7:12);'
    }, 'limo_p.sensors.odom.sample_time', {});

names = {'odom', 'imu', 'joints'};
blocks = {'Odometry', 'IMU', 'Joint states'};

for i = 1:3
    y = 58 + 85*(i-1);
    add_block('simulink/Sinks/Out1', [sensors_sys '/' names{i}], ...
        'Port', num2str(i), 'Position', [320 y 350 y+14]);
    add_line(sensors_sys, 'x/1', [blocks{i} '/1'], 'autorouting', 'on');
    add_line(sensors_sys, [blocks{i} '/1'], [names{i} '/1'], 'autorouting', 'on');
end


%% ========================================================================
%  LIMO (Gazebo) connections (once the subsystem ports exist)
%  ========================================================================

add_line(limo_sys, 'u/1', 'Command/1', 'autorouting', 'on');
add_line(limo_sys, 'Command/1', 'LIMO plant/1', 'autorouting', 'on');
add_line(limo_sys, 'LIMO plant/1', 'Command/2', 'autorouting', 'on');
add_line(limo_sys, 'LIMO plant/1', 'Sensors/1', 'autorouting', 'on');

add_line(limo_sys, 'Sensors/1', 'odom/1', 'autorouting', 'on');
add_line(limo_sys, 'Sensors/2', 'imu/1', 'autorouting', 'on');
add_line(limo_sys, 'Sensors/3', 'joints/1', 'autorouting', 'on');
add_line(limo_sys, 'LIMO plant/1', 'state/1', 'autorouting', 'on');
add_line(limo_sys, 'Command/1', 'tau/1', 'autorouting', 'on');
add_line(limo_sys, 'LIMO plant/2', 'contact/1', 'autorouting', 'on');

for sys = {limo_sys, steering_sys, cmd_vel_sys, efforts_sys, plant_sys, sensors_sys}
    Simulink.BlockDiagram.arrangeSystem(sys{1});
end


%% ========================================================================
%  Mask
%  ========================================================================

mask = Simulink.Mask.create(limo_sys);

mask.Type = 'AgileX LIMO (Gazebo)';
mask.Description = sprintf([ ...
    'AgileX LIMO with the dynamics of the ENSEM Gazebo simulation ' ...
    '(ROS 2 Foxy, limo_car/gazebo/ackermann_with_sensor.xacro).\n\n' ...
    'Input u:\n' ...
    '  Steering angle: [v (m/s); gamma (rad)], gamma = central steering ' ...
    'angle (tan(gamma) = L/R), positive to the left, limited to 30 deg\n' ...
    '  cmd_vel: [linear.x (m/s); angular.z (rad/s)] as on the ROS topic\n' ...
    '  Joint efforts: [tau_FL; tau_FR; tau_RL; tau_RR; ' ...
    'tau_delta_L; tau_delta_R] (N.m)\n\n' ...
    'Outputs: odom [X Y psi v_x v_y r], imu [q (4); omega (3); a (3)], ' ...
    'joints [omega (4); delta (2)], state (14), tau (6), ' ...
    'contact [F_x (4); F_y (4); F_n (4)].\n\n' ...
    'The other parameters are in limo.parameters() (+limo/parameters.m).']);
mask.Help = 'web(fullfile(fileparts(which(''limo_lib'')), ''README.md''))';

tabs = mask.addDialogControl('Type', 'tabcontainer', 'Name', 'tabs');
tabs.addDialogControl('Type', 'tab', 'Name', 'tab_command', 'Prompt', 'Command');
tabs.addDialogControl('Type', 'tab', 'Name', 'tab_pose', 'Prompt', 'Initial pose');
tabs.addDialogControl('Type', 'tab', 'Name', 'tab_contact', 'Prompt', 'Contact');

mask.addParameter( ...
    'Type', 'popup', ...
    'Name', 'input_mode', ...
    'Prompt', 'Input u:', ...
    'TypeOptions', { ...
        'Steering angle: [v; gamma]', ...
        'cmd_vel: [v; yaw rate] (twist adapter)', ...
        'Joint efforts (own controller)'}, ...
    'Value', 'Steering angle: [v; gamma]', ...
    'Evaluate', 'on', ...
    'Tunable', 'off', ...
    'Container', 'tab_command');

mask.addParameter( ...
    'Type', 'popup', ...
    'Name', 'pose_mode', ...
    'Prompt', 'Initial pose:', ...
    'TypeOptions', {'Origin', 'Room 207 spawn', 'Custom'}, ...
    'Value', 'Origin', ...
    'Evaluate', 'on', ...
    'Tunable', 'off', ...
    'Container', 'tab_pose', ...
    'Callback', strjoin({ ...
        'custom = Simulink.Mask.get(gcb).getParameter(''custom_pose'');'
        'if strcmp(get_param(gcb, ''pose_mode''), ''Custom'')'
        '    custom.Enabled = ''on'';'
        'else'
        '    custom.Enabled = ''off'';'
        'end'
        }, newline));

mask.addParameter( ...
    'Type', 'edit', ...
    'Name', 'custom_pose', ...
    'Prompt', 'Custom pose [X (m), Y (m), psi (rad)]:', ...
    'Value', '[0 0 0]', ...
    'Evaluate', 'on', ...
    'Tunable', 'off', ...
    'Enabled', 'off', ...
    'Container', 'tab_pose');

mask.addParameter( ...
    'Type', 'edit', ...
    'Name', 'mu', ...
    'Prompt', 'Friction coefficient mu (Gazebo: 1):', ...
    'Value', '1', ...
    'Evaluate', 'on', ...
    'Tunable', 'off', ...
    'Container', 'tab_contact');

mask.addParameter( ...
    'Type', 'edit', ...
    'Name', 'v_eps', ...
    'Prompt', 'Friction regularisation velocity (m/s):', ...
    'Value', '1e-3', ...
    'Evaluate', 'on', ...
    'Tunable', 'off', ...
    'Container', 'tab_contact');

mask.Initialization = strjoin({ ...
    '% Parameters of the LIMO, with the choices of the mask'
    'limo_p = limo.parameters();'
    ''
    'switch pose_mode'
    '    case 1'
    '        limo_p.initial.pose = [0; 0; 0];'
    '    case 2'
    '        limo_p.initial.pose = limo_p.initial.room207_pose;'
    '    otherwise'
    '        limo_p.initial.pose = custom_pose(:);'
    'end'
    ''
    'limo_p.contact.friction.mu = mu;'
    'limo_p.contact.friction.regularization_velocity = v_eps;'
    ''
    '% Block icon, stored next to limo_lib.slx'
    'limo_icon = fullfile(fileparts(which(''limo_lib'')), ''images'', ''limo_icon.png'');'
    }, newline);

% The icon area keeps the 1.39 aspect ratio of the picture on the
% default 280 x 200 block.
mask.IconUnits = 'normalized';
mask.RunInitForIconRedraw = 'on';
mask.Display = strjoin({ ...
    'image(limo_icon, [0.22 0.16 0.52 0.52]);'
    'switch input_mode'
    '    case 1'
    '        port_label(''input'', 1, ''v, \gamma'', ''texmode'', ''on'');'
    '    case 2'
    '        port_label(''input'', 1, ''cmd_vel'');'
    '    otherwise'
    '        port_label(''input'', 1, ''tau'');'
    'end'
    'port_label(''output'', 1, ''odom'');'
    'port_label(''output'', 2, ''imu'');'
    'port_label(''output'', 3, ''joints'');'
    'port_label(''output'', 4, ''state'');'
    'port_label(''output'', 5, ''tau'');'
    'port_label(''output'', 6, ''contact'');'
    }, newline);

set_param(lib, 'EnableLBRepository', 'on');
save_system(lib, fullfile(folder, [lib '.slx']));

fprintf('Saved %s\n', fullfile(folder, [lib '.slx']));


%% ========================================================================
%  Demo model
%  ========================================================================

new_system(model);

set_param(model, ...
    'Solver', 'ode15s', ...
    'StopTime', '10', ...
    'RelTol', '1e-6', ...
    'AbsTol', '1e-8', ...
    'MaxStep', 'auto', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SignalLogging', 'off', ...
    'SaveTime', 'off', ...
    'SaveOutput', 'off', ...
    'SaveState', 'off', ...
    'SaveFormat', 'Dataset');

% Make the library and the +limo package available
set_param(model, 'PostLoadFcn', ...
    'addpath(fileparts(get_param(bdroot, ''FileName'')));');

add_block('simulink/Sources/Clock', [model '/Clock'], ...
    'Position', [40 185 70 215]);

add_matlab_function([model '/Scenario'], [110 170 230 230], { ...
    'function u = scenario(t)'
    '%SCENARIO Speed and steering angle as a function of time.'
    '%   u = [v (m/s); gamma (rad)], gamma > 0 to the left'
    ''
    'v = 0;'
    'gamma = 0;'
    ''
    'if t >= 0.5'
    '    v = 0.5;'
    'end'
    ''
    'if t >= 3 && t < 8'
    '    gamma = 0.3;'
    'end'
    ''
    'u = [v; gamma];'
    }, '-1', {});

add_block(limo_sys, [model '/LIMO'], 'Position', [300 100 580 300]);

add_line(model, 'Clock/1', 'Scenario/1', 'autorouting', 'on');
add_line(model, 'Scenario/1', 'LIMO/1', 'autorouting', 'on');

for i = 1:numel(limo_outputs)
    y = 60 + 50*i;
    add_block('simulink/Sinks/To Workspace', [model '/' limo_outputs{i}], ...
        'VariableName', limo_outputs{i}, ...
        'SaveFormat', 'Timeseries', ...
        'Position', [680 y 760 y+30]);
    add_line(model, sprintf('LIMO/%d', i), [limo_outputs{i} '/1'], ...
        'autorouting', 'on');
end

save_system(model, fullfile(folder, [model '.slx']));
close_system(model, 0);
close_system(lib, 0);

fprintf('Saved %s\n', fullfile(folder, [model '.slx']));


%% ========================================================================
%  Local functions
%  ========================================================================

function add_subsystem(path, position)
%ADD_SUBSYSTEM Add an empty subsystem.

add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

end


function add_choice(path, position, condition)
%ADD_CHOICE Add a variant choice with the ports u, x -> tau.

add_subsystem(path, position);

add_block('simulink/Sources/In1', [path '/u'], ...
    'Position', [30 43 60 57]);
add_block('simulink/Sources/In1', [path '/x'], ...
    'Port', '2', 'Position', [30 108 60 122]);
add_block('simulink/Sinks/Out1', [path '/tau'], ...
    'Position', [520 68 550 82]);

set_param(path, 'VariantControl', condition);

end


function add_plugin_choice(path, position, condition, converter, converter_code, converter_parameters)
%ADD_PLUGIN_CHOICE Add a variant choice: command converter + Gazebo plugin.
%
%   The converter turns u into the /cmd_vel_gazebo command of the plugin.

add_choice(path, position, condition);

add_matlab_function([path '/' converter], [110 35 230 85], ...
    converter_code, '-1', converter_parameters);

add_matlab_function([path '/Gazebo Ackermann drive plugin'], [290 40 450 110], { ...
    'function tau = plugin(cmd_gazebo, x, limo_p)'
    '%PLUGIN gazebo_ros_ackermann_drive + Joint::SetForce(), run every physics step.'
    '%'
    '%   The plugin updates every limo_p.gazebo.plugin.sample_time and, as in'
    '%   Gazebo, its efforts act during the next physics step only.'
    ''
    'persistent pid step'
    ''
    'if isempty(pid)'
    '    pid = zeros(3, 2);'
    '    step = 0;'
    'end'
    ''
    'n = round(limo_p.gazebo.plugin.sample_time/limo_p.gazebo.physics_step);'
    ''
    '% No update at load time (no time elapsed yet)'
    'if step > 0 && mod(step, n) == 0'
    '    [tau_cmd, pid] = limo.gazebo_ackermann_plugin(cmd_gazebo, x([10 11 12]), pid, limo_p);'
    '    tau = limo.gazebo_set_force(tau_cmd, x(13:14), limo_p);'
    'else'
    '    tau = zeros(6, 1);'
    'end'
    ''
    'step = step + 1;'
    }, 'limo_p.gazebo.physics_step', {'limo_p'});

add_line(path, 'u/1', [converter '/1'], 'autorouting', 'on');
add_line(path, [converter '/1'], 'Gazebo Ackermann drive plugin/1', 'autorouting', 'on');
add_line(path, 'x/1', 'Gazebo Ackermann drive plugin/2', 'autorouting', 'on');
add_line(path, 'Gazebo Ackermann drive plugin/1', 'tau/1', 'autorouting', 'on');

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
