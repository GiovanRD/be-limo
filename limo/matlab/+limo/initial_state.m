function x0 = initial_state(p)
%INITIAL_STATE Initial state of limo.plant_dynamics (robot at rest).
%
%   x0 = limo.initial_state(p)
%
%   The pose is taken from p.initial.pose = [X; Y; psi]. All velocities
%   and steering angles are zero.

x0 = [
    p.initial.pose(:)
    zeros(11,1)
];

end
