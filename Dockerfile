# syntax=docker/dockerfile:1
FROM osrf/ros:noetic-desktop-full AS build
ENV DEBIAN_FRONTEND=noninteractive

# Add APT package deps
RUN apt-get update --fix-missing && apt-get upgrade -y && \
    apt-get install -y software-properties-common && \
    source /opt/ros/noetic/setup.bash \
    source /root/.bashrc && \ 
    rosdep update && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    apt update && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u && \
    apt-get install -y \
    libxcb-cursor0 fluid espeak sox python3-pyudev \
    python3-osrf-pycommon ros-noetic-librealsense2 python-is-python3 ros-noetic-realsense2-description gfortran \
    libhidapi-dev subversion ros-noetic-tf2-tools ros-noetic-realsense2-camera cmake-qt-gui \
    libopenigtlink-dev wget libtool librealsense2-dev libcppunit-dev \
    python3-pip libqt5xmlpatterns5-dev qtcreator librealsense2-dkms python3-catkin-tools \
    ros-noetic-libpointmatcher librealsense2-utils libudev-dev python3-rosinstall-generator libncurses5-dev \
    cmake-curses-gui git libbluetooth-dev python3-vcstool ros-noetic-cv-camera \
    swig libfox-1.6-dev

# Add Python deps
RUN pip3 install \
    testresources 'zipp>=3.1.0' pyrealsense2 'numpy>=1.20.3,<1.27.0' scipy \
    matplotlib ipython jupyter pandas sympy \
    nose 

# Add libnabo
RUN cd /root && \
    mkdir Libraries && \
    cd Libraries && \
    git clone --depth 1 https://github.com/ethz-asl/libnabo.git && \ 
    cd libnabo && \
    mkdir build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo /root/Libraries/libnabo && \
    make && make install

# Add libpointmatcher
RUN cd /root/Libraries && \
    git clone --depth 1 https://github.com/ethz-asl/libpointmatcher.git && \ 
    cd libpointmatcher && \
    mkdir build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo /root/Libraries/libpointmatcher && \
    make && make install

# Add dvrk
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cd /root/catkin_ws/src && \
    git clone --depth 1 https://github.com/ethz-asl/ethzasl_icp_mapping && \
    vcs import --recursive --workers 1 --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros1-dvrk-2.2.1.vcs && \
    catkin config  --extend /opt/ros/noetic && \
    catkin build --summary

# Add irob-saf
WORKDIR /root/catkin_ws/
RUN mkdir src/irob-saf
COPY . ./src/irob-saf
RUN cd /root/catkin_ws/ && catkin build irob-saf

FROM ros:foxy-ros1-bridge-focal
ENV DEBIAN_FRONTEND=noninteractive

COPY --from=build /root/catkin_ws /root/catkin_ws

RUN apt update && apt install -y ros-noetic-desktop-full python3-pip apt-transport-https

# Add librealsense
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt update && \
    apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

RUN pip3 install scipy pyrealsense2 cv-bridge

# Add ROS2 ports
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws/src && \
    git clone https://github.com/anderudp/irob-saf-ros2 && \
    cd /root/ros2_ws/ && colcon build --symlink-install

# Single quotes around EOF so the variables don't get mangled
COPY --chmod=777 <<'EOF' /ros_entrypoint.sh
#!/bin/bash

# Unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO

# Check if an argument was provided
if [ -z "$1" ]; then
    echo "No argument provided. Please provide 'none', 'ros1', 'ros2', or 'bridge'."
    exit 1
fi

rosver="$1"

case "$rosver" in
    none)
        echo "Neither ROS version is sourced."
        ;;
    ros1)
        echo "ROS1 is sourced."
        source "/root/catkin_ws/devel/setup.bash"
        ;;
    ros2)
        echo "ROS2 is sourced."
        source "/opt/ros/foxy/setup.bash"
        source "/root/ros2_ws/install/setup.bash"
        ;;
    bridge)
        echo "ROS1 and ROS2 are sourced."
        source "/root/catkin_ws/devel/setup.bash"
        source "/opt/ros/foxy/setup.bash"
        source "/root/ros2_ws/install/setup.bash"
        export ROS_MASTER_URI=http://localhost:11311
        ;;
    *)
        echo "Invalid argument. Please provide 'none', 'ros1', 'ros2', or 'bridge'."
        exit 1
        ;;
esac
bash
EOF

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["none"]
