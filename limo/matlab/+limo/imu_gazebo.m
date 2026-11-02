function [imu, v_imu] = imu_gazebo(x, v_imu_last, p)
%IMU_GAZEBO IMU measurement as computed by the Gazebo 11 IMU sensor.
%
%   [imu, v_imu] = limo.imu_gazebo(x, v_imu_last, p)
%
%   Inputs:
%       x          - Plant state (see limo.plant_dynamics)
%       v_imu_last - World velocity of the IMU at the previous sample [m/s]
%                    (2x1, use the initial velocity before the first call)
%
%   Outputs:
%       imu   - [q_x; q_y; q_z; q_w;          orientation (world reference)
%                omega_x; omega_y; omega_z;   angular velocity [rad/s]
%                a_x; a_y; a_z]               linear acceleration [m/s^2]
%       v_imu - World velocity of the IMU, to pass to the next call
%
%   As in ImuSensor::UpdateImpl(), the linear acceleration is the finite
%   difference of the IMU velocity over one sample, expressed in the IMU
%   frame, minus gravity (a resting IMU measures +g on z). The Gazebo
%   model defines no noise. The IMU is on the yaw axis of base_link, so
%   its planar velocity equals the base_link velocity.

dt = p.sensors.imu.sample_time;
g = p.environment.gravity;

psi = x(3);
c = cos(psi);
s = sin(psi);

% IMU velocity in the world frame
v_imu = [
    c*x(4) - s*x(5)
    s*x(4) + c*x(5)
];

% Acceleration in the world frame, rotated into the IMU frame
a_world = (v_imu - v_imu_last(:))/dt;
a_body = [
     c*a_world(1) + s*a_world(2)
    -s*a_world(1) + c*a_world(2)
];

imu = [
    0
    0
    sin(psi/2)
    cos(psi/2)
    0
    0
    x(6)
    a_body
    g
];

end
