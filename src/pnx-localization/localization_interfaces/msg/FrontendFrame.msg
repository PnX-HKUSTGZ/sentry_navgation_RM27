# One completed scan. All geometry is already deskewed by Point-LIO.
# instance_id changes at process startup; epoch changes at estimator reset.
string instance_id
uint64 epoch
uint64 sequence
# Pose: camera_init <- IMU body. Twist: IMU body.
# pose.covariance: [world-additive position, body-right rotation], NOT a
# standard ROS fixed-axis covariance. The backend must convert explicitly.
nav_msgs/Odometry odometry
# World geometry: cloud.header.frame_id == odometry.header.frame_id.
# Both timestamps must match exactly. Transform once by pose.inverse().
sensor_msgs/PointCloud2 cloud
bool imu_saturated
uint64 rejected_imu_updates
uint64 effective_points
uint64 lidar_updates
