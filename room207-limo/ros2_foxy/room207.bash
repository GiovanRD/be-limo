# Room 207 shortcuts for the ROS 2 Foxy workstation.
#
# Source this file from Bash, e.g. at the end of ~/.bashrc:
#
#     source ~/ros2_ws/src/room207-limo/ros2_foxy/room207.bash
#
# Then run r207_help to list the commands.
#
# Settings (environment variables, set them before sourcing to change them):
#     R207_WS             ROS 2 workspace                 (default ~/ros2_ws)
#     R207_ROBOT_SUBNET   network prefix of the robots    (default 100.75.155)
#     R207_ROBOT_USER     user on the robots              (default jetson)
#     R207_ROBOT          address of the robot in use     (set by r207_robot)
#     ROS_DOMAIN_ID       ROS 2 domain                    (default 0)

R207_WS="${R207_WS:-$HOME/ros2_ws}"
R207_ROBOT_SUBNET="${R207_ROBOT_SUBNET:-100.75.155}"
R207_ROBOT_USER="${R207_ROBOT_USER:-jetson}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"


# ============================================================
# Help
# ============================================================

r207_help() {
  cat <<'EOF'
Room 207 commands

  Workstation
    r207_env              source ROS 2 Foxy and the workspace
    r207_build            build room207_limo and its dependencies

  Simulation (Gazebo)
    r207_sim [args]       Gazebo + RViz (args: world:=..., spawn_z:=...)
    r207_demo             complete demonstration: simulation, recorder, control window
    r207_matlab           Gazebo + RViz only, to be driven by room207_gazebo.slx in MATLAB

  Real robot (run each command in its own terminal)
    r207_robot <N|IP>     select the robot: LIMO number N or full address
    r207_robot_ssh        open a shell on the robot
    r207_robot_start      start the LIMO driver on the robot
    r207_robot_server     start the TCP server (robot_communication) on the robot
    r207_robot_tunnel     open the SSH tunnel to the TCP server
    r207_robot_teleop     open the teleoperation window

  MATLAB (Windows or Linux): in room207-limo/matlab, run room207_setup,
  then open room207_demo.slx or room207_gazebo.slx.
EOF
}


# ============================================================
# Workstation
# ============================================================

r207_env() {
  if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
    echo "ROS 2 Foxy is not installed (/opt/ros/foxy)." >&2
    return 1
  fi

  source /opt/ros/foxy/setup.bash

  if [[ -f "$R207_WS/install/setup.bash" ]]; then
    source "$R207_WS/install/setup.bash"
  else
    echo "Workspace not built yet: run r207_build." >&2
  fi
}

r207_build() {
  source /opt/ros/foxy/setup.bash || return 1

  (
    cd "$R207_WS" &&
      colcon build --symlink-install --packages-up-to room207_limo
  ) || return 1

  r207_env
}


# ============================================================
# Simulation
# ============================================================

r207_sim() {
  r207_env || return 1
  ros2 launch room207_limo room207_limo.launch.py "$@"
}

r207_demo() {
  r207_env || return 1
  ros2 launch room207_limo demo_simu.launch.py "$@"
}

r207_matlab() {
  r207_env || return 1
  echo "Gazebo for MATLAB: run room207_gazebo.slx with ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
  echo "Do not start the control window: it would also publish on /cmd_vel_gazebo."
  ros2 launch room207_limo room207_limo.launch.py "$@"
}


# ============================================================
# Real robot
# ============================================================

r207_robot() {
  if [[ $# != 1 ]]; then
    echo "Usage: r207_robot <LIMO number | address>" >&2
    return 2
  fi

  if [[ "$1" =~ ^[0-9]+$ ]]; then
    export R207_ROBOT="$R207_ROBOT_SUBNET.$1"
  else
    export R207_ROBOT="$1"
  fi

  echo "Robot: $R207_ROBOT_USER@$R207_ROBOT"
}

_r207_robot_address() {
  if [[ -z "${R207_ROBOT:-}" ]]; then
    echo "No robot selected: run r207_robot <LIMO number>." >&2
    return 1
  fi

  echo "$R207_ROBOT_USER@$R207_ROBOT"
}

r207_robot_ssh() {
  local address
  address="$(_r207_robot_address)" || return 1
  ssh "$address"
}

r207_robot_start() {
  # "start" is the alias installed on the robot by limo/configuration/limo_ubuntu20.04_foxy
  local address
  address="$(_r207_robot_address)" || return 1
  ssh -t "$address" "bash -ic start"
}

r207_robot_server() {
  # The LIMO driver publishes its odometry on /odom
  local address
  address="$(_r207_robot_address)" || return 1
  ssh -t "$address" \
    "bash -ic 'ros2 run robot_communication tcp_server --ros-args -p odom_topic:=/odom'"
}

r207_robot_tunnel() {
  local address
  address="$(_r207_robot_address)" || return 1
  echo "Tunnel to the TCP server of $address (Ctrl+C to close)."
  ssh -N -L 8765:127.0.0.1:8765 "$address"
}

r207_robot_teleop() {
  r207_env || return 1
  ros2 run teleoperation teleop_limo_real
}
