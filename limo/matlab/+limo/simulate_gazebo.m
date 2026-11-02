function out = simulate_gazebo(command, t_end, p, options)
%SIMULATE_GAZEBO Simulate the LIMO Gazebo model in MATLAB (no Simulink).
%
%   out = limo.simulate_gazebo(command, t_end, p)
%   out = limo.simulate_gazebo(command, t_end, p, 'Command', 'steering')
%   out = limo.simulate_gazebo(..., 'OdeOptions', odeset(...))
%
%   Inputs:
%       command - Function handle t -> [v; w] giving the command:
%                   'cmd_vel'  (default) w = yaw rate, /cmd_vel angular.z
%                              [rad/s], converted by the twist adapter
%                   'steering' w = steering angle gamma [rad], see
%                              limo.gazebo_steering_command
%                 v is the linear velocity [m/s] in both cases.
%       t_end   - Simulation duration [s]
%       p       - Parameters from limo.parameters()
%
%   Options:
%       'Command'    - 'cmd_vel' or 'steering'
%       'OdeOptions' - Options of ode15s (default RelTol 1e-6, AbsTol 1e-8)
%
%   Output structure, sampled at the plugin rate (100 Hz):
%       t    - Time [s]                                   (N x 1)
%       x    - Plant state, see limo.plant_dynamics      (N x 14)
%       cmd  - /cmd_vel_gazebo command [linear.x angular.z] (N x 2)
%       tau  - Efforts given to SetForce() at each update   (N x 6)
%       odom - [X Y psi v_x v_y r] ground-truth odometry    (N x 6)
%       imu  - IMU message, see limo.imu_gazebo            (N x 10)
%
%   Chain reproduced at each plugin update (every 10 ms):
%       command -> twist adapter or steering command -> Ackermann plugin
%               -> SetForce()
%   As in Gazebo, the efforts act during one physics step (1 ms).
%
%   This function is the reference implementation of the LIMO (Gazebo)
%   block of limo_lib.slx. It is slower than Simulink.

arguments
    command (1,1) function_handle
    t_end (1,1) double {mustBePositive}
    p (1,1) struct
    options.Command (1,1) string {mustBeMember(options.Command, ["cmd_vel", "steering"])} = "cmd_vel"
    options.OdeOptions (1,1) struct = odeset('RelTol', 1e-6, 'AbsTol', 1e-8)
end

ode_options = options.OdeOptions;

Ts = p.gazebo.plugin.sample_time;
dt_phys = p.gazebo.physics_step;

N = floor(t_end/Ts + 1e-9) + 1;
t = (0:N-1).'*Ts;

out.t = t;
out.x = zeros(N, 14);
out.cmd = zeros(N, 2);
out.tau = zeros(N, 6);
out.odom = zeros(N, 6);
out.imu = zeros(N, 10);

x = limo.initial_state(p);
pid = zeros(3, 2);
v_imu = [0; 0];

for k = 1:N

    % Sensors
    out.x(k,:) = x.';
    out.odom(k,:) = x(1:6).';
    [out.imu(k,:), v_imu] = limo.imu_gazebo(x, v_imu, p);

    if k == N
        break
    end

    % Command chain. The plugin skips its first call (no time elapsed
    % since load), like OnUpdate() in Gazebo.
    v_w = command(t(k));

    if options.Command == "steering"
        cmd = limo.gazebo_steering_command(v_w(1), v_w(2));
    else
        cmd = limo.gazebo_twist_adapter(v_w(1), v_w(2), p);
    end

    if k > 1
        [tau_cmd, pid] = limo.gazebo_ackermann_plugin(cmd, x([10 11 12]), pid, p);
        tau = limo.gazebo_set_force(tau_cmd, x(13:14), p);
    else
        tau = zeros(6, 1);
    end

    out.cmd(k,:) = cmd.';
    out.tau(k,:) = tau.';

    % Physics
    x = integrate(x, tau, t(k), t(k) + dt_phys, p, ode_options);
    x = integrate(x, zeros(6,1), t(k) + dt_phys, t(k+1), p, ode_options);
end

end


function x = integrate(x, u, t0, t1, p, ode_options)
%INTEGRATE Integrate the plant with constant joint efforts.

[~, xs] = ode15s(@(~, xx) limo.plant_dynamics(xx, u, p), [t0, t1], x, ode_options);
x = xs(end,:).';

end
