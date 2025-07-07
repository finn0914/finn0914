# station_pointcloud

## Run

```
ros2 launch station_pointcloud station_pointcloud.launch.xml
```

### Parameters

Parameters such as point cloud max z depth and camera intrinsics are in `launch/station_pointcloud.launch.yaml`.

## RViz2

In case we need to run rviz2 to see the result of the recorded bag file, play back the bag file using
```
ros2 bag play <bag_directory> --clock
```
optionally with `--loop`. And then visualize using
```
rviz2 --ros-args -p use_sim_time:=true
```
# ground-station-ROS2 Version

## Build

Always build the code with the below command 
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select realsense2_camera --allow-overriding realsense2_camera
```
To build station_tcp_connection and station_pointcloud it's needed to install below libraries first.
```
sudo apt install ros-humble-theora-image-transport

```
and 
```
sudo apt install ros-humble-pcl-ros

```
