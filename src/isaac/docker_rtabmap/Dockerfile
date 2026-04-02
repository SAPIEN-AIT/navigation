###########################################
# MINA RTABMap SLAM + Nav2 image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia

ARG DEBIAN_FRONTEND=noninteractive

# ── Keys fix (same as main image) ──
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg2 lsb-release software-properties-common

## ROS 2
RUN mkdir -p /usr/share/keyrings \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor --batch --yes --no-tty \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list

# ── Install RTABMap + Nav2 + ZED deps ──
RUN add-apt-repository ppa:borglab/gtsam-release-4.1
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-depthimage-to-laserscan \
    ros-${ROS_DISTRO}-test-msgs \
    python3-zmq \
    libyaml-cpp-dev \
    libgtsam-dev \
    libgtsam-unstable-dev

# ── USB rules for ZED camera ──
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

USER $USERNAME

# ── Copy source code ──
WORKDIR /home/$USERNAME/dev_ws/src
COPY --chown=xplore:xplore . .

WORKDIR /home/$USERNAME/dev_ws

# ── Install ROS 2 dependencies ──
RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

ENV ZED_DIR=/usr/local/zed
ENV CMAKE_PREFIX_PATH=/usr/local/zed

# ── Source workspace on login ──
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# ── Install ZED SDK ──
RUN sudo chmod +x /home/$USERNAME/dev_ws/src/scripts/zed_setup.sh \
    && /bin/bash /home/$USERNAME/dev_ws/src/scripts/zed_setup.sh /home/$USERNAME/dev_ws/src/scripts/install-zed-x86_64.sh

# ── Build ZED wrapper + RTABMap package ──
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-up-to zed2i_rtabmap"

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Resolve calibration symlink to a real file (source gets overlaid by volume mount at runtime)
RUN if [ -L /usr/local/zed/settings/SN38462249.conf ]; then \
      sudo cp -fL /usr/local/zed/settings/SN38462249.conf /tmp/SN38462249.conf && \
      sudo mv /tmp/SN38462249.conf /usr/local/zed/settings/SN38462249.conf; \
    fi
