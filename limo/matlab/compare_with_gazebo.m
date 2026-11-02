function result = compare_with_gazebo(bag_folder)
%COMPARE_WITH_GAZEBO Compare the MATLAB model with a rosbag recorded in Gazebo.
%
%   result = compare_with_gazebo(bag_folder)
%
%   Record the bag in Gazebo with the robot at rest at the start, e.g.:
%
%       ros2 bag record /cmd_vel /odom /joint_states /limo/imu
%
%   The /cmd_vel commands of the bag are replayed through
%   limo.simulate_gazebo() from the first /odom pose, and the simulated
%   odometry and steering angles are plotted against the recorded ones.
%
%   Requires the ROS Toolbox (ros2bagreader).
%
%   Output structure:
%       t        - Simulation time from the first /odom message [s]
%       gazebo   - Recorded [X Y psi v_x v_y r] interpolated on t
%       matlab   - Simulated [X Y psi v_x v_y r]
%       rms      - RMS error of [v_x r] over the record

folder = fileparts(mfilename('fullpath'));
addpath(folder);

p = limo.parameters();

bag = ros2bagreader(bag_folder);


%% ========================================================================
%  Odometry (header stamps are in simulation time)
%  ========================================================================

[odom_msgs, odom_bag_time] = read_topic(bag, '/odom');

n = numel(odom_msgs);
odom_time = zeros(n, 1);
odom = zeros(n, 6);

for i = 1:n
    m = odom_msgs{i};
    q = m.pose.pose.orientation;
    odom_time(i) = stamp(m.header.stamp);
    odom(i,:) = [
        m.pose.pose.position.x
        m.pose.pose.position.y
        atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2))
        m.twist.twist.linear.x
        m.twist.twist.linear.y
        m.twist.twist.angular.z
    ].';
end

odom(:,3) = unwrap(odom(:,3));

% /cmd_vel has no header: map the bag (wall-clock) time to the
% simulation time with the /odom messages.
time_map = polyfit(odom_bag_time, odom_time, 1);


%% ========================================================================
%  Commands
%  ========================================================================

[cmd_msgs, cmd_bag_time] = read_topic(bag, '/cmd_vel');

cmd_time = polyval(time_map, cmd_bag_time);
cmd = zeros(numel(cmd_msgs), 2);

for i = 1:numel(cmd_msgs)
    cmd(i,:) = [cmd_msgs{i}.linear.x, cmd_msgs{i}.angular.z];
end

t0 = odom_time(1);
cmd_time = cmd_time - t0;
odom_time = odom_time - t0;

cmd_vel = @(t) command_at(t, cmd_time, cmd, p.gazebo.adapter.command_timeout);


%% ========================================================================
%  Simulation
%  ========================================================================

p.initial.pose = odom(1,1:3).';

t_end = odom_time(end);
out = limo.simulate_gazebo(cmd_vel, t_end, p);

gazebo = interp1(odom_time, odom, out.t);

result.t = out.t;
result.gazebo = gazebo;
result.matlab = out.odom;
result.rms = sqrt(mean((out.odom(:,[4 6]) - gazebo(:,[4 6])).^2, 1, 'omitnan'));

fprintf('RMS error: v_x = %.4f m/s, yaw rate = %.4f rad/s\n', result.rms);


%% ========================================================================
%  Steering joints (if recorded)
%  ========================================================================

steering = [];

if any(bag.AvailableTopics.Row == "/joint_states")
    [js_msgs, ~] = read_topic(bag, '/joint_states');
    steering = nan(numel(js_msgs), 3);

    for i = 1:numel(js_msgs)
        m = js_msgs{i};
        names = string(m.name);
        iL = find(names == "front_left_steer_joint", 1);
        iR = find(names == "front_right_steer_joint", 1);

        if ~isempty(iL) && ~isempty(iR)
            steering(i,:) = [stamp(m.header.stamp) - t0, m.position(iL), m.position(iR)];
        end
    end

    steering = steering(~isnan(steering(:,1)),:);
end


%% ========================================================================
%  Plots
%  ========================================================================

figure('Name', 'Gazebo vs MATLAB - trajectory');
plot(gazebo(:,1), gazebo(:,2), out.odom(:,1), out.odom(:,2), 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
legend('Gazebo', 'MATLAB');

figure('Name', 'Gazebo vs MATLAB - motion');
tiledlayout(3, 1);

nexttile;
plot(out.t, gazebo(:,4), out.t, out.odom(:,4), 'LineWidth', 1.5);
grid on;
ylabel('v_x [m/s]');
legend('Gazebo', 'MATLAB');

nexttile;
plot(out.t, gazebo(:,6), out.t, out.odom(:,6), 'LineWidth', 1.5);
grid on;
ylabel('r [rad/s]');

nexttile;
plot(out.t, out.x(:,11:12), 'LineWidth', 1.5);
hold on;
if ~isempty(steering)
    plot(steering(:,1), steering(:,2:3), '--', 'LineWidth', 1.5);
    legend('\delta_L MATLAB', '\delta_R MATLAB', '\delta_L Gazebo', '\delta_R Gazebo');
else
    legend('\delta_L MATLAB', '\delta_R MATLAB');
end
grid on;
xlabel('Time [s]');
ylabel('\delta [rad]');

end


function [msgs, bag_time] = read_topic(bag, topic)
%READ_TOPIC Messages and bag receive times of one topic.

selection = select(bag, 'Topic', topic);
msgs = readMessages(selection);
bag_time = selection.MessageList.Time;

end


function t = stamp(s)
%STAMP builtin_interfaces/Time to seconds.

t = double(s.sec) + 1e-9*double(s.nanosec);

end


function c = command_at(t, cmd_time, cmd, timeout)
%COMMAND_AT Last /cmd_vel command received before t.
%
%   Zero before the first command and, as in the twist adapter, when no
%   command was received during the timeout.

i = find(cmd_time <= t, 1, 'last');

if isempty(i) || t - cmd_time(i) > timeout
    c = [0; 0];
else
    c = cmd(i,:).';
end

end
