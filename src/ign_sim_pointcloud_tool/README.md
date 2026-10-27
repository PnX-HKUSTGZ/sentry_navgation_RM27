# isaac_sim_pointcloud_tool

This package converts pointcloud in Ignition Gazebo to velodyne format. This is a ROS package. It subscribes to the LiDAR rostopic which is published by Ignition Gazebo. And it republish a LiDAR rostopoic in Velodyne format.

Some SLAM algorithm needs pointcloud in Velodyne format so that it can extract corner points. But Isaac ROS only send pointcloud contains XYZ information. This package helps to convert pointcloud to velodyne format.

Gazebo GPU lidar returns are instantaneous. `instantaneous_scan` defaults to
`true`, so every output point has `time=0` and the input header is preserved.
Non-finite returns are discarded. `instantaneous_scan=false` retains the old
synthetic 0–100 ms offsets only for reproducing earlier experiments.

Point-LIO must also use `preprocess.instantaneous_scan=true`, which is supplied
by the simulation configuration and implemented by the reviewed isolated build
in `tools/global_relocalization/build_mapping.bash`. Otherwise its missing-time
fallback infers a spinning scan from azimuth even when all times are zero.
