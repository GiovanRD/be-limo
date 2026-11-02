function tests = test_gazebo_model
%TEST_GAZEBO_MODEL Tests of the Gazebo-equivalent LIMO model.
%
%   Run from the matlab folder with:
%
%       results = runtests('tests/test_gazebo_model.m')

tests = functiontests(localfunctions);

end


function setupOnce(testCase)

folder = fileparts(fileparts(mfilename('fullpath')));
addpath(folder);

testCase.TestData.folder = folder;
testCase.TestData.p = limo.parameters();

end


%% ========================================================================
%  Parameters
%  ========================================================================

function test_mass_properties(testCase)

p = testCase.TestData.p;

% 4.34 chassis + 0.21 sensors + 2 x 0.25 knuckles + 4 x 0.5 wheels
verifyEqual(testCase, p.mass.total, 7.05, 'AbsTol', 1e-12);

% Wheel spin inertia of the URDF cylinder: m(3r^2 + h^2)/12
verifyEqual(testCase, p.wheel.inertia_rotation.rear, 3.375e-4, 'RelTol', 1e-12);

% Contact track: collision centres 22.5 mm outboard of the joints
verifyEqual(testCase, p.geometry.contact_track, 0.213, 'AbsTol', 1e-12);

end


function test_rest_equilibrium(testCase)

p = testCase.TestData.p;

[xdot, aux] = limo.plant_dynamics(limo.initial_state(p), zeros(6,1), p);

verifyEqual(testCase, xdot, zeros(14,1), 'AbsTol', 1e-12);
verifyEqual(testCase, sum(aux.F_n), p.mass.total*p.environment.gravity, ...
    'RelTol', 1e-12);

end


%% ========================================================================
%  Gazebo plugin
%  ========================================================================

function test_twist_adapter(testCase)

p = testCase.TestData.p;

% Standard conversion
cmd = limo.gazebo_twist_adapter(0.5, 0.6, p);
verifyEqual(testCase, cmd, [0.5; atan(0.24*0.6/0.5)], 'AbsTol', 1e-12);

% Reverse: angle computed with |v|
cmd = limo.gazebo_twist_adapter(-0.5, 0.6, p);
verifyEqual(testCase, cmd, [-0.5; atan(0.24*0.6/0.5)], 'AbsTol', 1e-12);

% Saturation and dead band
cmd = limo.gazebo_twist_adapter(0.1, 5, p);
verifyEqual(testCase, cmd(2), atan(0.24/0.55), 'AbsTol', 1e-12);
verifyEqual(testCase, limo.gazebo_twist_adapter(0.0005, 1, p), [0; 0]);

end


function test_plugin_first_update(testCase)

p = testCase.TestData.p;

% At rest, 0.5 m/s and no steering
[tau, pid] = limo.gazebo_ackermann_plugin([0.5; 0], [0; 0; 0], zeros(3,2), p);

% e = 0 - 0.5/R; cmd = -(P e + D (e - 0)/dt), integral clamped to 0
e = -0.5/p.geometry.wheel_radius;
expected = -(0.5*e + 0.001*e/0.01);

verifyEqual(testCase, tau, [0; 0; expected; expected; 0; 0], 'AbsTol', 1e-12);
verifyEqual(testCase, pid(1,:), [e, 0], 'AbsTol', 1e-12);

end


function test_plugin_steering_targets(testCase)

p = testCase.TestData.p;

% With the joints at their targets and no previous error, the steering
% efforts are zero.
angle = 0.3;
t = tan(angle);
W = 0.213;
L = 0.24;
left = atan2(t, 1 - W/2/L*t);
right = atan2(t, 1 + W/2/L*t);

pid = zeros(3,2);
tau = limo.gazebo_ackermann_plugin([0.5; angle], [0; left; right], pid, p);

verifyEqual(testCase, tau(5:6), [0; 0], 'AbsTol', 1e-9);

% In reverse the steering direction flips
pid = zeros(3,2);
tau = limo.gazebo_ackermann_plugin([-0.5; angle], [0; -right; -left], pid, p);

verifyEqual(testCase, tau(5:6), [0; 0], 'AbsTol', 1e-9);

end


function test_set_force_limits(testCase)

p = testCase.TestData.p;

% Effort limit of the steering joints, wheels unlimited
tau = limo.gazebo_set_force([0; 0; 100; 100; 100; -100], [0; 0], p);
verifyEqual(testCase, tau, [0; 0; 100; 100; 20; -20]);

% Velocity limit: no effort pushing beyond 3 rad/s
tau = limo.gazebo_set_force([0; 0; 0; 0; 10; 10], [3.5; -3.5], p);
verifyEqual(testCase, tau(5:6), [0; 10]);

end


%% ========================================================================
%  Simulation
%  ========================================================================

function test_straight_line(testCase)

p = testCase.TestData.p;

out = limo.simulate_gazebo(@(t) [0.5; 0], 3, p);

% Reaches the commanded speed (no rolling resistance, no I term)
verifyEqual(testCase, out.x(end,4), 0.5, 'AbsTol', 1e-3);

% Stays on the x axis
verifyLessThan(testCase, max(abs(out.x(:,2))), 1e-6);
verifyLessThan(testCase, max(abs(out.x(:,3))), 1e-6);

% The IMU measures the acceleration of the robot and +g on z
verifyEqual(testCase, out.imu(end,10), p.environment.gravity, 'AbsTol', 1e-12);
verifyEqual(testCase, out.imu(end,8), 0, 'AbsTol', 1e-2);

end


function test_turn(testCase)

p = testCase.TestData.p;

out = limo.simulate_gazebo(@(t) [0.5; 0.6], 6, p);

k = out.t >= 3;

% The yaw rate follows the command within 15 % on average
verifyEqual(testCase, mean(out.x(k,6)), 0.6, 'RelTol', 0.15);

% The robot turns left
verifyGreaterThan(testCase, out.x(end,3), 1);

end


function test_steering_command(testCase)

p = testCase.TestData.p;

% The plugin flips angular.z in reverse; the command compensates it
verifyEqual(testCase, limo.gazebo_steering_command(0.5, 0.3), [0.5; 0.3]);
verifyEqual(testCase, limo.gazebo_steering_command(-0.5, 0.3), [-0.5; -0.3]);
verifyEqual(testCase, limo.gazebo_steering_command(0, 0.3), [0; 0.3]);

% Forward: the yaw rate is close to the bicycle model v tan(gamma)/L
gamma = 0.3;
out = limo.simulate_gazebo(@(t) [0.5; gamma], 5, p, 'Command', 'steering');
k = out.t >= 2;

verifyEqual(testCase, mean(out.x(k,6)), ...
    mean(out.x(k,4))*tan(gamma)/p.geometry.wheelbase, 'RelTol', 0.15);

% Reverse: the wheels stay turned to the left, the robot turns right
out = limo.simulate_gazebo(@(t) [-0.3; gamma], 3, p, 'Command', 'steering');
k = out.t >= 1;

verifyGreaterThan(testCase, mean(out.x(k,11:12), 'all'), 0.2);
verifyLessThan(testCase, mean(out.x(k,6)), 0);

end


function test_simulink_matches_matlab(testCase)

folder = testCase.TestData.folder;
p = testCase.TestData.p;

if ~isfile(fullfile(folder, 'limo_gazebo.slx'))
    run(fullfile(folder, 'build_limo_gazebo_model.m'));
end

% Straight-line part of the default scenario (before the turn at 3 s)
simout = sim(fullfile(folder, 'limo_gazebo.slx'), 'StopTime', '2.9');

out = limo.simulate_gazebo(@(t) [0.5*(t >= 0.5); 0], 2.9, p, ...
    'Command', 'steering');

state = simout.state;
x = interp1(state.Time, squeeze(state.Data).', out.t);

verifyEqual(testCase, x(:,1:6), out.x(:,1:6), 'AbsTol', 1e-6);

end


function test_library_mask_options(testCase)

folder = testCase.TestData.folder;
p = testCase.TestData.p;

if ~isfile(fullfile(folder, 'limo_lib.slx'))
    run(fullfile(folder, 'build_limo_gazebo_model.m'));
end

% Joint-effort input and Room 207 spawn pose, chosen in the mask
model = 'test_limo_lib_mask';
new_system(model);
testCase.addTeardown(@() close_system(model, 0));

load_system(fullfile(folder, 'limo_lib.slx'));

add_block('simulink/Sources/Constant', [model '/tau'], ...
    'Value', '[0; 0; 0.02; 0.02; 0; 0]');
add_block('limo_lib/LIMO (Gazebo)', [model '/LIMO'], ...
    'input_mode', 'Joint efforts (own controller)', ...
    'pose_mode', 'Room 207 spawn');
add_block('simulink/Sinks/To Workspace', [model '/state'], ...
    'VariableName', 'state', 'SaveFormat', 'Timeseries');

add_line(model, 'tau/1', 'LIMO/1');
add_line(model, 'LIMO/4', 'state/1');

for i = [1 2 3 5 6]
    name = sprintf('T%d', i);
    add_block('simulink/Sinks/Terminator', [model '/' name]);
    add_line(model, sprintf('LIMO/%d', i), [name '/1']);
end

set_param(model, 'Solver', 'ode15s', 'StopTime', '1', ...
    'ReturnWorkspaceOutputs', 'on');

simout = sim(model);
x = squeeze(simout.state.Data).';

% Starts at the Room 207 spawn pose
verifyEqual(testCase, x(1,1:3).', p.initial.room207_pose, 'AbsTol', 1e-12);

% A constant rear torque pushes the robot forward, i.e. along +Y there
verifyGreaterThan(testCase, x(end,2) - x(1,2), 0.01);
verifyEqual(testCase, x(end,1), x(1,1), 'AbsTol', 1e-4);

end
