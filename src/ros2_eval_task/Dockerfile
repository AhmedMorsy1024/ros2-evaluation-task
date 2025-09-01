# ---- Base: ROS 2 Humble Desktop (Ubuntu 22.04) ----
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl wget nano lsb-release \
    mesa-utils mesa-utils-extra \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
    ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-visualization-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME}
USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN mkdir -p /home/${USERNAME}/ros2_ws/src
WORKDIR /home/${USERNAME}/ros2_ws

SHELL ["/bin/bash", "-lc"]

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

CMD ["bash"]

