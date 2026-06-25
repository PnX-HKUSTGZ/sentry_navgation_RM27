# relocalization_manager

Relocalization manager for the RM27 navigation stack. It uses
[small_gicp](https://github.com/koide3/small_gicp.git) as an internal point
cloud verification backend, while the manager node owns relocalization state and
publishes the final `map -> odom` transform.

Given a registered point cloud in the odom frame and a prior point cloud mapped
using [point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) or
similar tools, `RelocalizationManagerNode` calculates the correction between the
two point clouds. `SmallGicpVerifier` only returns a verified candidate pose; it
does not decide or publish TF directly.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## Build

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

## Usage

1. Set the prior point cloud file through `prior_pcd_file` in the bringup launch
   file or in [launch/relocalization_manager_launch.py](launch/relocalization_manager_launch.py)
   when running this package alone.

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch relocalization_manager relocalization_manager_launch.py
    ```
