function tests = test_room207
%TEST_ROOM207 Tests of the Room 207 Simulink demonstrations.
%
%   Run from the matlab folder with:
%
%       results = runtests('tests/test_room207.m')
%
%   The ROS 2 connection itself needs a running Gazebo demonstration and
%   is not tested here; test_gazebo_model_compiles only checks the model.

tests = functiontests(localfunctions);

end


function setupOnce(testCase)

folder = fileparts(fileparts(mfilename('fullpath')));
addpath(folder);
room207_setup();

if ~isfile(fullfile(folder, 'room207_demo.slx')) || ...
        ~isfile(fullfile(folder, 'room207_gazebo.slx'))
    run(fullfile(folder, 'build_room207_models.m'));
end

testCase.TestData.folder = folder;

end


function teardownOnce(~)

close(findall(0, 'Type', 'figure', 'Name', 'ROOM 207 | LIMO'));

end


function test_keyboard_command(testCase)

d = room207.parameters();
room207.viewer('start', 'no_model', {'LIMO'});

verifyEqual(testCase, room207.viewer('command'), [0; 0]);

% Z + Q: forward, steering left
room207.viewer('key', 'press', 'z');
room207.viewer('key', 'press', 'q');
verifyEqual(testCase, room207.viewer('command'), ...
    [d.teleop.max_speed; d.teleop.max_steering]);

% Releasing Q keeps the speed
room207.viewer('key', 'release', 'q');
verifyEqual(testCase, room207.viewer('command'), [d.teleop.max_speed; 0]);

% Z + S cancel each other, as in the ROS 2 widget
room207.viewer('key', 'press', 's');
verifyEqual(testCase, room207.viewer('command'), [0; 0]);

% Space releases everything
room207.viewer('key', 'press', 'd');
room207.viewer('key', 'press', 'space');
verifyEqual(testCase, room207.viewer('command'), [0; 0]);

end


function test_demo_scenario(testCase)

folder = testCase.TestData.folder;
d = room207.parameters();
p = limo.parameters();

model = 'room207_demo';
load_system(fullfile(folder, [model '.slx']));
testCase.addTeardown(@() close_system(model, 0));

set_param([model '/Command source/Keyboard or scenario'], 'sw', '0');

simout = sim(model, 'StopTime', '16', 'EnablePacing', 'off');
y = squeeze(simout.odom.Data).';

% Starts at the Room 207 spawn pose
verifyEqual(testCase, y(1,1:3).', p.initial.room207_pose, 'AbsTol', 1e-9);

% Turns left into the car park ...
verifyGreaterThan(testCase, max(y(:,3)), 2.5);

% ... stays on the map ...
verifyLessThan(testCase, max(abs(y(:,1))), d.map.width/2);
verifyLessThan(testCase, max(abs(y(:,2))), d.map.height/2);

% ... and reverses back into the parking bay
verifyLessThan(testCase, norm(y(end,1:2) - y(1,1:2)), 0.15);

end


function test_gazebo_model_compiles(testCase)

folder = testCase.TestData.folder;

model = 'room207_gazebo';
load_system(fullfile(folder, [model '.slx']));
testCase.addTeardown(@() close_system(model, 0));

set_param(model, 'SimulationCommand', 'update');

verifyEqual(testCase, get_param([model '/LIMO'], 'input_mode'), ...
    'Steering angle: [v; gamma]');

end
