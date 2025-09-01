FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    pkg-config \
    python3-colcon-common-extensions \
    python3-vcstool \
    \
    # ROS 2 deps
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-srvs \
    ros-humble-rcl-interfaces \
    ros-humble-message-filters \
    ros-humble-rosidl-default-generators \
    \
    # Libraries
    libeigen3-dev \
    libopencv-dev \
    libboost-serialization-dev \
 && rm -rf /var/lib/apt/lists/*

ARG REPO_URL=https://github.com/One-star11/orb_slam_2_ros.git

# Create workspace and clone repo
ENV WS /ws
RUN mkdir -p $WS/src && \
    git clone --depth 1 ${REPO_URL} $WS/src/orb_slam_2_ros
WORKDIR $WS

# Overlay local context (ensures ROS2 files present even if fork doesn't have them yet)
COPY . $WS/src/orb_slam_2_ros

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint (relative to this Dockerfile directory)
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


