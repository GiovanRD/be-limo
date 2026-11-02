function [tau, pid] = gazebo_ackermann_plugin(cmd_gazebo, joints, pid, p)
%GAZEBO_ACKERMANN_PLUGIN One update of the Gazebo Ackermann drive plugin.
%
%   [tau, pid] = limo.gazebo_ackermann_plugin(cmd_gazebo, joints, pid, p)
%
%   Inputs:
%       cmd_gazebo - Last /cmd_vel_gazebo message [linear.x; angular.z]
%       joints     - Measured joints [omega_RR; delta_L; delta_R]
%       pid        - PID memory, 3x2 matrix of [pErrLast, iErr] rows:
%                    row 1 linear velocity, row 2 left steering,
%                    row 3 right steering.
%                    Use zeros(3,2) before the first update.
%       p          - Parameters from limo.parameters()
%
%   Outputs:
%       tau - Joint efforts given to Joint::SetForce()
%             [tau_FL; tau_FR; tau_RL; tau_RR; tau_delta_L; tau_delta_R]
%       pid - Updated PID memory
%
%   MATLAB version of GazeboRosAckermannDrivePrivate::OnUpdate()
%   (gazebo_ros_pkgs, branch foxy). It must be called every
%   p.gazebo.plugin.sample_time seconds:
%
%       - The rear wheels receive the same torque, computed from the
%         velocity of the rear right wheel only. The front wheels are free.
%       - angular.z is a steering angle, multiplied by sign(linear.x).
%       - The left/right angles follow the Ackermann rule with the
%         wheel separation measured between the collision centres.
%       - The PIDs are gazebo::common::PID: the command is
%         -(P*e + I*int(e) + D*de/dt) with e = actual - target, and the
%         integral term is clamped to the <i_range> interval.

%% Parameters

pp = p.gazebo.plugin;
dt = pp.sample_time;

R_w = pp.wheel_radius;
W = pp.wheel_separation;
L = pp.wheel_base;


%% Commands

target_linear = min(max(cmd_gazebo(1), -pp.max_speed), pp.max_speed);

% copysign(1.0, x) returns +1 for x = 0
target_rot = cmd_gazebo(2)*sign_nonzero(cmd_gazebo(1));
target_rot = min(max(target_rot, -pp.max_steer), pp.max_steer);


%% Velocity loop (rear right wheel)

linear_diff = joints(1) - target_linear/R_w;
[linear_cmd, pid(1,:)] = gazebo_pid(linear_diff, dt, ...
    pp.linear_pid, pp.linear_i_range, pid(1,:));


%% Steering loops

tan_steer = tan(target_rot);

target_left  = atan2(tan_steer, 1 - W/2/L*tan_steer);
target_right = atan2(tan_steer, 1 + W/2/L*tan_steer);

[left_cmd, pid(2,:)] = gazebo_pid(joints(2) - target_left, dt, ...
    pp.steering_pid, pp.steering_i_range, pid(2,:));

[right_cmd, pid(3,:)] = gazebo_pid(joints(3) - target_right, dt, ...
    pp.steering_pid, pp.steering_i_range, pid(3,:));


%% Joint efforts

tau = [
    0
    0
    linear_cmd
    linear_cmd
    left_cmd
    right_cmd
];

end


function [cmd, state] = gazebo_pid(err, dt, gains, i_range, state)
%GAZEBO_PID gazebo::common::PID::Update() (Gazebo 11).
%
%   state = [pErrLast, iErr]

p_gain = gains(1);
i_gain = gains(2);
d_gain = gains(3);
i_min = i_range(1);
i_max = i_range(2);

p_err_last = state(1);
i_err = state(2);

p_term = p_gain*err;

i_err = i_err + dt*err;
i_term = i_gain*i_err;

if i_term > i_max
    i_term = i_max;
    i_err = i_term/i_gain;
elseif i_term < i_min
    i_term = i_min;
    i_err = i_term/i_gain;
end

d_err = (err - p_err_last)/dt;
d_term = d_gain*d_err;

% The plugin does not set command limits, so no clamping
cmd = -p_term - i_term - d_term;

state = [err, i_err];

end


function s = sign_nonzero(x)
%SIGN_NONZERO copysign(1.0, x) for a finite x.

if x < 0
    s = -1;
else
    s = 1;
end

end
