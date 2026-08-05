FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# The build network's IPv6 is advertised but unreachable, so apt hangs for
# minutes per host on SYN timeouts to the (unreachable) AAAA records before
# falling back to IPv4. Force IPv4 to skip that entirely.
RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

# Gazebo classic 11 + ROS-Gazebo bridge and the extra deps the packages under
# src/ import at runtime (cv_bridge, tf2, keyboard teleop, opencv, numpy).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros \
        ros-humble-gazebo-plugins \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        ros-humble-tf2-ros \
        ros-humble-xacro \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        x11-apps \
        mesa-utils \
        iproute2 \
        iputils-ping \
        net-tools \
        && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pynput opencv-python numpy

# The launch files under src/sim_pkg/launch resolve world/model/plugin paths
# via os.path.expanduser('~/bfmc_sim/...'), so the workspace MUST live at
# $HOME/bfmc_sim inside the container (mounted there by docker-compose).
ENV WORKSPACE=/root/bfmc_sim
WORKDIR $WORKSPACE

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f $WORKSPACE/install/setup.bash ] && source $WORKSPACE/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
