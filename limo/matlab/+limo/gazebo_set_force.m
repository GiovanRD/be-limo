function tau = gazebo_set_force(tau_cmd, delta_dot, p)
%GAZEBO_SET_FORCE Joint effort truncation of Joint::SetForce() (Gazebo 11).
%
%   tau = limo.gazebo_set_force(tau_cmd, delta_dot, p)
%
%   Inputs:
%       tau_cmd   - Commanded efforts
%                   [tau_FL; tau_FR; tau_RL; tau_RR; tau_delta_L; tau_delta_R]
%       delta_dot - Steering joint rates when the force is set [rad/s]
%
%   Output:
%       tau       - Efforts applied during the next physics step
%
%   Joint::CheckAndTruncateForce() sets the effort to zero when the joint
%   already exceeds its velocity limit and the effort pushes further, and
%   then clamps it to the effort limit. Only the steering joints have
%   limits; the wheel joints are continuous without <limit>.

tau = tau_cmd(:);

rate_max = p.steering.rate_max;
torque_max = p.steering.torque_max;

for j = 1:2
    k = 4 + j;

    if delta_dot(j) > rate_max && tau(k) > 0
        tau(k) = 0;
    elseif delta_dot(j) < -rate_max && tau(k) < 0
        tau(k) = 0;
    end

    tau(k) = min(max(tau(k), -torque_max), torque_max);
end

end
