# LIMO MATLAB/Simulink Models

This directory contains MATLAB scripts and Simulink models for simulating, analyzing, and developing controllers for the LIMO mobile robot.

The `LIMO (Gazebo)` block of the library `limo_lib.slx` reproduces the LIMO simulated in Gazebo (ROS 2 Foxy, `limo_car/gazebo/ackermann_with_sensor.xacro`). It has the same masses, inertias, geometry, wheel–ground friction, Ackermann drive plugin, and IMU and odometry sensors. A controller developed against this model sees the same robot as in the Gazebo simulation, without a ROS installation.

## Requirements

- MATLAB and Simulink R2026b
- ROS Toolbox, only for `compare_with_gazebo.m`

## Quick start

Add this folder to the MATLAB path, then run the demo:

```matlab
addpath('limo/matlab')
run_limo_gazebo          % simulates limo_gazebo.slx and plots
```

`limo_gazebo.slx` is a demo model: a `Scenario` block drives a link to the library block. To change the command, edit the `Scenario` block. It returns `[v (m/s); γ (rad)]` as a function of time.

To use the LIMO in your own model, drag the `LIMO (Gazebo)` block from the Library Browser (section **LIMO (ENSEM)**), or from `limo_lib.slx`.

## Block mask

Double-click the `LIMO (Gazebo)` block to choose:

| Tab | Parameter | Choices |
|---|---|---|
| Command | Input `u` | **Steering angle** (default): `u = [v; γ]`, linear velocity (m/s) and steering angle (rad), sent to the Gazebo plugin. **cmd_vel**: `u = [linear.x; angular.z]` (yaw rate), as published on the ROS `/cmd_vel` topic, converted by the twist adapter. **Joint efforts (own controller)**: `u = [tau_FL; tau_FR; tau_RL; tau_RR; tau_delta_L; tau_delta_R]` in N.m, applied directly to the plant. |
| Initial pose | Initial pose | **Origin**, **Room 207 spawn** (pose of the `room207-limo` launch file) or **Custom** `[X Y psi]` |
| Contact | Friction coefficient | μ = 1 in Gazebo |
| Contact | Regularisation velocity | 1 mm/s by default. Smaller is closer to Gazebo but slower to simulate. |

γ is the steering angle of the equivalent central wheel (bicycle model, `tan γ = L/R`, L = 0.24 m), positive to the left, also when driving backwards. The plugin limits it to ±30° and computes the left and right wheel angles itself. Beyond about 25°, the inner wheel reaches its 30° joint stop. In `cmd_vel` mode the twist adapter computes `γ = atan(L ω_z / |v|)` and limits it to 23.6°.

The input port label shows the selected command (`v, γ`, `cmd_vel` or `tau`). All the other parameters come from `+limo/parameters.m`, which the block reloads at every simulation start. The mask values take precedence over that file.

The block outputs are the same in both modes: `odom`, `imu`, `joints`, `state`, `tau`, `contact` (see below).

Without Simulink, the same model runs in MATLAB:

```matlab
p = limo.parameters();
out = limo.simulate_gazebo(@(t) [0.5; 0.3*(t > 2)], 10, p, 'Command', 'steering');
plot(out.odom(:,1), out.odom(:,2)); axis equal
```

## Model structure

```text
limo_lib.slx
└── LIMO (Gazebo)                        masked block
    ├── Command                          variant subsystem, chosen in the mask
    │   ├── Steering angle
    │   │   ├── Steering command         [v; γ] -> plugin command
    │   │   └── Gazebo Ackermann drive plugin  100 Hz PID + Joint::SetForce(), 1 kHz block
    │   ├── cmd_vel
    │   │   ├── Twist adapter            gazebo_twist_adapter.py
    │   │   └── Gazebo Ackermann drive plugin  100 Hz PID + Joint::SetForce(), 1 kHz block
    │   └── Joint efforts                u passed to the plant
    ├── LIMO plant                       joint efforts -> 14 states
    └── Sensors                          odometry, IMU, joint states (100 Hz)
```

The blocks only call functions of the `+limo` package. All equations live in that package:

| Function | Role |
|---|---|
| `limo.parameters` | Gazebo-derived parameters, with the source of every value |
| `limo.plant_dynamics` | Equations of motion: `xdot = f(x, u)` |
| `limo.gazebo_steering_command` | `[v; γ]` → `/cmd_vel_gazebo` conversion (sign handling in reverse) |
| `limo.gazebo_twist_adapter` | `/cmd_vel` → `/cmd_vel_gazebo` conversion |
| `limo.gazebo_ackermann_plugin` | One update of `gazebo_ros_ackermann_drive` (Foxy), with the Gazebo PID |
| `limo.gazebo_set_force` | Effort and velocity limits applied by `Joint::SetForce()` |
| `limo.imu_gazebo` | Gazebo 11 IMU (finite-difference acceleration) |
| `limo.simulate_gazebo` | MATLAB-only simulation, the reference implementation for the Simulink model |
| `limo.initial_state` | Initial state from `p.initial.pose` |

`build_limo_gazebo_model.m` generates `limo_lib.slx` and `limo_gazebo.slx`. Re-run it after changing the blocks or the mask instead of editing the `.slx` files by hand. The block icon is `images/limo_icon.png`, which must stay next to `limo_lib.slx`.

### State, outputs and logged signals

The plant state is `x = [X; Y; psi; v_x; v_y; r; omega_FL; omega_FR; omega_RL; omega_RR; delta_L; delta_R; delta_dot_L; delta_dot_R]`, where `(v_x, v_y)` is the velocity of `base_link` in the body frame.

The simulation logs the following timeseries in the `Simulink.SimulationOutput`:

| Signal | Content |
|---|---|
| `odom` | `[X Y psi v_x v_y r]` of `base_link`, ground truth as published by the plugin |
| `imu` | `[q_x q_y q_z q_w omega_x omega_y omega_z a_x a_y a_z]` |
| `joints` | `[omega_FL omega_FR omega_RL omega_RR delta_L delta_R]` |
| `state` | Plant state `x` |
| `tau` | Joint efforts applied to the plant |
| `contact` | `[F_x (4); F_y (4); F_n (4)]` tyre forces in the wheel frames, order FL FR RL RR |

## What the model reproduces from Gazebo

These points come from the xacro files and from the Gazebo 11, `gazebo_ros_pkgs` (Foxy) and SDF 1.7 sources. Several of them differ from what the xacro tags suggest.

- **Masses and inertias.** The total mass is 7.05 kg: chassis, sensors lumped by Gazebo, knuckles and wheels. The wheel spin inertia is `m(3r^2+h^2)/12 = 3.4e-4 kg.m^2`, because the URDF inertia is not rotated with the cylinder.
- **Contact geometry.** The collision cylinders are offset by 22.5 mm outboard of the wheel joints. The contacts are therefore 0.213 m apart, while the steering pivots are 0.168 m apart.
- **Friction.** μ = 1 (ODE default, no `<surface>` tags) with the cone friction model selected by the plugin. ODE enforces exact sticking. The model uses a regularized friction cone, with `p.contact.friction.regularization_velocity` = 1 mm/s.
- **Drive.** Only the rear wheels are driven. Both receive the same torque, computed from the rear-right wheel speed alone. The front wheels roll freely.
- **Plugin gains.** The Gazebo PID returns `-(P e + I ∫e + D de/dt)` with `e = actual - target`. The `i_range` of `0 0` disables both integral terms. The first update produces a derivative kick.
- **Ignored tags.** `max_wheel_torque`, `max_wheel_acceleration`, `wheel_separation` and `wheel_diameter` are not read by the Foxy plugin. The drive torque is unbounded.
- **Ackermann geometry.** The plugin computes the left/right angles with the separation between the collision centers (0.213 m), not between the pivots. The front wheels therefore slightly fight each other in turns. This scrub slows the robot down in turns.
- **One-step efforts.** `Joint::SetForce()` acts during one physics step (1 ms), and the plugin runs at 100 Hz. The efforts are pulses of 1 ms every 10 ms.
- **Steering joints.** Limits of ±30°, 20 N.m and 3 rad/s (the velocity limit cancels efforts that push further), damping 0.1, Coulomb friction 0.1.
- **IMU.** The linear acceleration is the finite difference of the velocity over one sample, minus gravity (g = 9.8). The Gazebo model has no noise.
- **Odometry.** Ground-truth pose and velocity of the model, not wheel odometry.

### Expected behavior

- **Speed.** In a straight line the robot reaches 96 % of a 0.5 m/s command 0.5 s after the step. In a turn it is slower than commanded because of the front-wheel scrub: about 0.43 m/s for 0.5 m/s at 0.6 rad/s.
- **Steering chatter.** After a steering change, the steering PID (Kp = 10000, saturated at 20 N.m and applied as 1 ms pulses) falls into a limit cycle of a few degrees. The trajectory during and after turns is therefore sensitive to small numerical differences. Compare runs on averages (speed, yaw rate, radius), not sample by sample.

### Differences with Gazebo

- **Planar motion.** Chassis pitch, roll and heave are neglected. The normal loads follow the static balance plus a quasi-static load transfer. Gazebo's rigid four-point contact is statically indeterminate, and the model splits the lateral transfer in proportion to the axle loads.
- **Contact.** Each wheel touches the ground at one point, the collision center. ODE may generate several points along the cylinder line, which adds a small scrub torque on the steering.
- **Friction.** The friction cone is regularized. Stick is approximated by a very steep slope, not enforced exactly as in ODE's iterative LCP solver.
- **Stops.** The steering joint stops are stiff spring-dampers.
- **Out of scope.** The laser and depth camera are not modelled. The spawn drop from `spawn_z` and the 0.5 s command timeout of the twist adapter are not modelled in Simulink. `compare_with_gazebo` applies the timeout when replaying a bag.

## Validation against Gazebo

Record a bag in Gazebo, starting with the robot at rest:

```bash
ros2 bag record /cmd_vel /odom /joint_states /limo/imu
```

Then replay it in MATLAB:

```matlab
result = compare_with_gazebo('path/to/bag_folder');
```

The script replays the recorded `/cmd_vel` through `limo.simulate_gazebo` from the first `/odom` pose. It plots the trajectory, speed, yaw rate and steering angles of both simulations, and prints the RMS errors.

## Tests

```matlab
results = runtests('tests/test_gazebo_model.m')
```

The tests check the parameters, the plugin, adapter and `SetForce` logic against hand-computed values, and the straight-line and turning behavior. They also check that the Simulink model matches `limo.simulate_gazebo`.

## Other models

`limo.dynamics`, `limo.controller` and `tests/test_straight_line.m` are a simpler model with perfect wheel–ground adhesion. They share `limo.parameters()`.

## Safety

These models are intended for development and simulation. Validate controller behavior in simulation before testing on a physical LIMO, and follow appropriate safety procedures when connecting software to hardware.
