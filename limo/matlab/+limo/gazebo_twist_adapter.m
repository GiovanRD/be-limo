function cmd_gazebo = gazebo_twist_adapter(v, omega_z, p)
%GAZEBO_TWIST_ADAPTER Convert a standard Twist into the Gazebo plugin command.
%
%   cmd_gazebo = limo.gazebo_twist_adapter(v, omega_z, p)
%
%   Inputs:
%       v       - Linear velocity command, /cmd_vel linear.x [m/s]
%       omega_z - Yaw rate command, /cmd_vel angular.z [rad/s]
%
%   Output:
%       cmd_gazebo = [linear.x; angular.z] published on /cmd_vel_gazebo
%
%   MATLAB version of limo_car/scripts/gazebo_twist_adapter.py.
%
%   The Foxy Ackermann plugin interprets angular.z as a steering angle
%   and multiplies it by sign(linear.x). The adapter therefore sends
%
%       angle = atan(L*omega_z/|v|)
%
%   saturated to max_steering_angle. Commands with |v| <= 1 mm/s stop
%   the robot. The command timeout of the ROS node is not modelled.

L = p.gazebo.adapter.wheelbase;
angle_max = p.gazebo.adapter.max_steering_angle;

if ~isfinite(v) || ~isfinite(omega_z) || abs(v) <= p.gazebo.adapter.min_speed
    cmd_gazebo = [0; 0];
    return
end

angle = atan(L*omega_z/abs(v));
angle = min(max(angle, -angle_max), angle_max);

cmd_gazebo = [v; angle];

end
