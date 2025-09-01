# ROS2 Evaluation Task - Battery Model Spawner

> **📋 For complete documentation, setup instructions, and demonstration videos, see the [main README.md](../../README.md) in the workspace root.**

## Quick Launch

```bash
# Complete system with model spawner and camera capture
ros2 launch ros2_eval_task gazebo_with_spawner.launch.py

# Basic Gazebo environment only
ros2 launch ros2_eval_task gazebo.launch.py
```

## System Features

- ✅ Automated battery model spawning (4 different models)
- ✅ Camera integration with image capture
- ✅ Smart model management (delete previous instances)
- ✅ Random position generation
- ✅ Professional C++ implementation with async operations

## Architecture

- **`GazeboUtilsClient`** - Utility library for Gazebo operations
- **`ModelSpawnerNode`** - Main spawning and camera node
- **Battery Models** - 4 different battery types with meshes and textures

---

## Dependencies (Ubuntu 22.04 + ROS2 Humble)

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs \
  libopencv-dev
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_eval_task
source install/setup.bash
```
