# ros2_eval_task

## Dependencies (Native)
Make sure you have **ROS 2 Humble** and **Gazebo Classic** installed, then install the required packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs
```

## Build the Workspace (Native)
```bash
mkdir -p ~/ros2_ws/src
# place this package inside ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install && source install/setup.bash
```

## Quickstart (Native)
```bash
ros2 launch ros2_eval_task gazebo.launch.py
```

---

## Docker

### 1) Build the image
```bash
# from the pkg root (where the Dockerfile is)
docker build -t ros2-gz-classic:humble .
```

### 2) Run the container (with GUI)
```bash
docker run -it --name ros2_gz_gui \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --device /dev/dri:/dev/dri \
  -v ~/interview_ws:/home/dev/ros2_ws \
  -w /home/dev/ros2_ws \
  ros2-gz-classic:humble
```

### 3) Inside the container: source, build, and launch
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Make Gazebo see your package models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/dev/ros2_ws/src/ros2_eval_task/models

# Build
colcon build --symlink-install

# Overlay
source install/setup.bash

# Launch
ros2 launch ros2_eval_task gazebo.launch.py
```

### 4) Open another shell in the same container (optional)
```bash
docker exec -it ros2_gz_gui bash
```
