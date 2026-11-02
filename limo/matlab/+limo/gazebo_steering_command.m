function cmd_gazebo = gazebo_steering_command(v, gamma)
%GAZEBO_STEERING_COMMAND Gazebo plugin command from speed and steering angle.
%
%   cmd_gazebo = limo.gazebo_steering_command(v, gamma)
%
%   Inputs:
%       v     - Linear velocity [m/s]
%       gamma - Steering angle of the equivalent central wheel (bicycle
%               model, tan(gamma) = L/R), positive to the left [rad]
%
%   Output:
%       cmd_gazebo = [linear.x; angular.z] for /cmd_vel_gazebo
%
%   The Foxy Ackermann plugin reads angular.z as the steering angle and
%   multiplies it by copysign(1, linear.x). The angle is multiplied by the
%   same sign here, so that gamma keeps its physical meaning (wheels turned
%   to the left for gamma > 0) when driving backwards. The plugin limits
%   gamma to +/- max_steer (30 deg).

if v < 0
    cmd_gazebo = [v; -gamma];
else
    cmd_gazebo = [v; gamma];
end

end
