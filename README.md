# pointcloud_saver_ros

![Build Status](https://github.com/ToshikiNakamura0412/pointcloud_saver_ros/workflows/build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS package for saving pointcloud data as PCD file.

Save path is `~/.ros/map_cloud.pcd`.

## Environment
- Ubuntu 20.04
- ROS Noetic

## Install and Build
```bash
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/pointcloud_saver_ros.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic      # Install dependencies
catkin build pointcloud_saver_ros -DCMAKE_BUILD_TYPE=Release # Release build is recommended
```

## How to use
### Run
```bash
rosrun pointcloud_saver_ros pointcloud_saver cloud:=/your/pointcloud/topic
```
### Save
1. rqt
2. Plugins -> Services -> Service Caller
3. Call service `~<name>/request_save`

## Subscribed Topics
- ~cloud (`sensor_msgs/PointCloud2`)
  - The input pointcloud
