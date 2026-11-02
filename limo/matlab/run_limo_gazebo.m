%RUN_LIMO_GAZEBO Simulate limo_gazebo.slx and plot the results.
%
%   Edit the "Scenario" block of the model to change the command [v; gamma],
%   the mask of the LIMO block to change the input type, and
%   limo.parameters() to change the robot.

folder = fileparts(mfilename('fullpath'));
addpath(folder);

if ~isfile(fullfile(folder, 'limo_gazebo.slx'))
    build_limo_gazebo_model;
end

simout = sim(fullfile(folder, 'limo_gazebo.slx'));

p = limo.parameters();


%% ========================================================================
%  Signals
%  ========================================================================

odom = simout.odom;
t = odom.Time;
y_odom = squeeze(odom.Data).';

y_imu = squeeze(simout.imu.Data).';
y_joints = squeeze(simout.joints.Data).';


%% ========================================================================
%  Plots
%  ========================================================================

figure('Name', 'LIMO (Gazebo) - trajectory');
plot(y_odom(:,1), y_odom(:,2), 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
title('base\_link trajectory');

figure('Name', 'LIMO (Gazebo) - motion');
tiledlayout(4, 1);

nexttile;
plot(t, y_odom(:,4), t, y_odom(:,5), 'LineWidth', 1.5);
grid on;
ylabel('[m/s]');
legend('v_x', 'v_y');
title('base\_link velocity (body frame)');

nexttile;
plot(t, y_odom(:,6), 'LineWidth', 1.5);
grid on;
ylabel('[rad/s]');
title('Yaw rate');

nexttile;
plot(t, y_joints(:,5), t, y_joints(:,6), 'LineWidth', 1.5);
grid on;
ylabel('[rad]');
legend('\delta_L', '\delta_R');
title('Steering joints');

nexttile;
plot(t, y_joints(:,1:4)*p.geometry.wheel_radius, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('[m/s]');
legend('FL', 'FR', 'RL', 'RR');
title('Wheel peripheral speeds R\omega');

figure('Name', 'LIMO (Gazebo) - IMU');
plot(simout.imu.Time, y_imu(:,8:9), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('[m/s^2]');
legend('a_x', 'a_y');
title('IMU linear acceleration');
