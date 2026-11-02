function [odom_msg, odom_state] = odom_sensor(v_true, psi_dot_true, dt)
% ODOM_SENSOR Computes odometry via dead-reckoning (wheels + heading).
%
% Inputs:
%   v_true       : True longitudinal velocity [m/s]
%   psi_dot_true : True yaw rate [rad/s]
%   dt           : Odometry sampling time interval [s]
%
% Outputs:
%   odom_msg     : Structure containing the estimated pose [X, Y, theta] and velocities
%   odom_state   : Persistent internal state for integration (X, Y, theta)

    % Persistent state to accumulate position (Dead Reckoning)
    persistent X Y theta initialized;
    if isempty(initialized)
        X = 0.0;
        Y = 0.0;
        theta = 0.0;
        initialized = true;
    end

    % 1. Add realistic noise to wheel velocity / odometry sensors
    sigma_v = 0.01;   % Velocity noise [m/s]
    sigma_w = 0.005;  % Angular velocity noise [rad/s]

    v_measured     = v_true + sigma_v * randn;
    psi_dot_meas   = psi_dot_true + sigma_w * randn;

    % 2. Kinematic integration (Simplified differential/Ackermann odometry model)
    % Update orientation
    theta = theta + psi_dot_meas * dt;

    % Update position in the global frame (World)
    X = X + v_measured * cos(theta) * dt;
    Y = Y + v_measured * sin(theta) * dt;

    % 3. Construct output message (ROS 2 nav_msgs/Odometry compatible)
    odom_msg.x     = X;
    odom_msg.y     = Y;
    odom_msg.theta = theta;
    odom_msg.v     = v_measured;
    odom_msg.omega = psi_dot_meas;

    % Save state for the next step
    odom_state = [X; Y; theta];

end