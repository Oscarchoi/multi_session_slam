# ------------------------------------------------------------
# multi_session_slam + ndt_omp_ros2 – ROS 2 Foxy (Ubuntu 20.04)
# ------------------------------------------------------------
FROM ros:foxy-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# ---------- OS-level dependencies ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential                    \
    git curl wget                      \
    ros-foxy-test-msgs                 \
    ros-foxy-pcl-conversions           \
    ros-foxy-pcl-ros                   \
    ros-foxy-libg2o                    \
    python3-colcon-common-extensions   \
    libpcl-dev libflann-dev            \
    libboost-all-dev                   \
    && rm -rf /var/lib/apt/lists/*

# ---------- Workspace ----------
COPY . /ros2_ws/src/multi_session_slam
WORKDIR /ros2_ws/src

CMD ["/bin/bash"]
    