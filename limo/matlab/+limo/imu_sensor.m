function [imu_msg, imu_cov] = imu_sensor(ax_true, ay_true, az_true, gx_true, gy_true, gz_true, yaw_true)
% IMU_SENSOR Simulates the IMU with noise calibrated from C++ driver covariances.

    % Standard deviations derived from covariance matrices (square root)
    sigma_accel = sqrt(1.0);       % = 1.0
    sigma_gyro  = sqrt(1e-6);      % = 0.001
    sigma_orient= sqrt(1e-6);      % = 0.001

    % Persistent bias for the gyroscope (slight random walk)
    persistent bias_gx bias_gy bias_gz;
    if isempty(bias_gx)
        bias_gx = sigma_gyro * randn;
        bias_gy = sigma_gyro * randn;
        bias_gz = sigma_gyro * randn;
    end

    %% 1. Apply noise using C++ standard deviations
    ax = ax_true + sigma_accel * randn;
    ay = ay_true + sigma_accel * randn;
    az = az_true + sigma_accel * randn + 9.81; % Gravity on the Z-axis

    gx = gx_true + bias_gx + sigma_gyro * randn;
    gy = gy_true + bias_gy + sigma_gyro * randn;
    gz = gz_true + bias_gz + sigma_gyro * randn;

    %% 2. Orientation and Quaternion (with coherent noise on yaw)
    yaw_noisy = yaw_true + sigma_orient * randn;
    
    roll = 0.0; pitch = 0.0; yaw = yaw_noisy;
    qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);

    %% 3. Construct output structure
    imu_msg.linear_acceleration = [ax; ay; az];
    imu_msg.angular_velocity    = [gx; gy; gz];
    imu_msg.orientation         = [qx; qy; qz; qw];

    %% 4. Restore official C++ driver covariance matrices
    imu_cov.linear_acceleration = diag([1.0, 1.0, 1.0]);
    imu_cov.angular_velocity    = diag([1e-6, 1e-6, 1e-6]);
    imu_cov.orientation         = diag([1e-6, 1e-6, 1e-6]);

end