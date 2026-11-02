function c = control_parameters()
%CONTROL_PARAMETERS Controller parameters of the AgileX LIMO.
%
%   c = limo.control_parameters()

%% ========================================================================
% Longitudinal velocity controller
% ========================================================================

% PID gains (sized for a robot of ~2.15 kg with max 5 Nm torque)
c.velocity.Kp = 15.0;
c.velocity.Ki = 2.0;
c.velocity.Kd = 0.1;


%% ========================================================================
% Steering controller
% ========================================================================

% PID gains (sized to saturate cleanly at 2 Nm for an error > 0.1 rad)
c.steering.Kp = 5.0;
c.steering.Ki = 1;
c.steering.Kd = 0.1;


%% ========================================================================
% Controller execution
% ========================================================================

c.frequency = 100;                                  % [Hz]
c.sample_time = 1/c.frequency;                      % [s]


%% ========================================================================
% Command interface
% ========================================================================

c.command.topic = "/cmd_vel";
c.command.names = ["linear_velocity", "angular_velocity"];

end