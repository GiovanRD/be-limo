FROM ubuntu:20.04

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=foxy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential ca-certificates cmake curl git gnupg2 \
    less locales lsb-release nano pkg-config software-properties-common \
    sudo tree udev usbutils vim wget \
    libboost-program-options-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
    libgl1-mesa-dri libgl1-mesa-glx libglu1-mesa libopencv-dev \
    libturbojpeg0-dev libudev-dev libusb-1.0-0-dev libuvc-dev \
    mesa-utils \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 package repository
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS 2 Foxy and application dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-argcomplete python3-matplotlib python3-numpy python3-opencv \
    python3-pyqt5 python3-pytest python3-scipy python3-yaml \
    python3-dev python3-pip python3-setuptools \
    python3-rosdep swig \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-foxy-desktop \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-cartographer-ros \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-nav2-bringup \
    ros-foxy-robot-localization \
    ros-foxy-rqt-robot-steering \
    ros-foxy-slam-toolbox \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-twist-mux \
    ros-foxy-xacro \
    ros-foxy-camera-info-manager \
    ros-foxy-cv-bridge \
    ros-foxy-image-geometry \
    ros-foxy-image-publisher \
    ros-foxy-image-transport \
    ros-foxy-tf2-eigen \
    ros-foxy-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Prepare package manifests for dependency installation
COPY ros2_foxy/limo_ros2/ /tmp/workspace-src/limo_ros2/
COPY ros2_foxy/ros2_astra_camera/ /tmp/workspace-src/ros2_astra_camera/
COPY ros2_foxy/ydlidar_ros2_driver/ /tmp/workspace-src/ydlidar_ros2_driver/

# Install dependencies declared in package manifests and build YDLidar SDK
COPY ros2_foxy/YDLidar-SDK/ /tmp/ydlidar-sdk/

RUN rosdep init \
    && rosdep update --rosdistro foxy --include-eol-distros \
    && apt-get update \
    && rosdep install \
        --from-paths /tmp/workspace-src \
        --ignore-src \
        -y \
        --rosdistro foxy \
    && cmake -S /tmp/ydlidar-sdk -B /tmp/ydlidar-build \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TEST=OFF \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && cmake --build /tmp/ydlidar-build --parallel 2 \
    && cmake --install /tmp/ydlidar-build \
    && ldconfig \
    && rm -rf /tmp/workspace-src /tmp/ydlidar-sdk /tmp/ydlidar-build \
    && rm -rf /var/lib/apt/lists/*

# Development user
RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" \
        --create-home --shell /bin/bash "${USERNAME}" \
    && usermod -aG dialout,video,sudo "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && echo 'source /opt/ros/foxy/setup.bash' \
        >> "/home/${USERNAME}/.bashrc" \
    && echo 'if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then source "$HOME/ros2_ws/install/setup.bash"; fi' \
        >> "/home/${USERNAME}/.bashrc" \
    && mkdir -p "/home/${USERNAME}/ros2_ws" \
    && chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}/ros2_ws"

# Helper scripts
COPY configuration/wsl2_ubuntu20.04_foxy/ros-entrypoint.sh /usr/local/bin/ros-entrypoint
COPY configuration/wsl2_ubuntu20.04_foxy/workspace-deps.sh /usr/local/bin/workspace-deps
COPY configuration/wsl2_ubuntu20.04_foxy/collect-manifests.py /usr/local/bin/collect-manifests

RUN sed -i 's/\r$//' \
        /usr/local/bin/ros-entrypoint \
        /usr/local/bin/workspace-deps \
        /usr/local/bin/collect-manifests \
    && chmod 0755 \
        /usr/local/bin/ros-entrypoint \
        /usr/local/bin/workspace-deps \
        /usr/local/bin/collect-manifests

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
CMD ["bash"]