#include "li_initialization.h"
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false,
     refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0,
       online_calib_starts_time = 0.0;  //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::msg::Imu imu_last, imu_next;
// sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;
std::size_t dropped_lidar_frames = 0, dropped_imu_messages = 0;
bool imu_gap_detected = false;

namespace
{
void clear_sensor_buffers_locked()
{
  lidar_buffer.clear();
  time_buffer.clear();
  imu_deque.clear();
  ptr_con->clear();
  lidar_pushed = false;
  imu_pushed = false;
  frame_ct = 0;
}

void bound_lidar_queue_locked()
{
  while (lidar_buffer.size() > static_cast<std::size_t>(max_lidar_queue_size)) {
    lidar_buffer.pop_front();
    if (!time_buffer.empty()) {
      time_buffer.pop_front();
    }
    ++dropped_lidar_frames;
  }
}
}  // namespace

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar timestamp rolled back; reset buffers");
    clear_sensor_buffers_locked();
    last_timestamp_imu = -1.0;
    flg_reset = true;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
  // printf("check lidar time %f\n", last_timestamp_lidar);
  // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
  //     timediff_set_flg = true;
  //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
  //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
  // }

  if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));  //unit:s
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;  //msg->header.stamp.toSec();
      }
      for (int i = 0; i < ptr->size(); i++) {
        ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
        ptr_con->push_back(ptr->points[i]);
      }
      frame_ct++;
      if (frame_ct >= con_frame_num) {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;
        lidar_buffer.push_back(ptr_con_i);
        double time_con_i = time_con;
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      } else {
        ++dropped_lidar_frames;
        static rclcpp::Clock diagnostic_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("li_initialization"), diagnostic_clock,
          1000, "LiDAR frame empty after preprocessing: input_points=%u blind=%.3f m",
          msg->width * msg->height, p_pre->blind);
      }
    }
  }
  bound_lidar_queue_locked();
  if (scan_count < MAXN) {
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  }
  sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar timestamp rolled back; reset buffers");
    clear_sensor_buffers_locked();
    last_timestamp_imu = -1.0;
    flg_reset = true;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
  // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
  //     timediff_set_flg = true;
  //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
  //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
  // }

  if (cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));  //unit:s
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;  //msg->header.stamp.toSec();
      }
      for (int i = 0; i < ptr->size(); i++) {
        ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
        ptr_con->push_back(ptr->points[i]);
      }
      frame_ct++;
      if (frame_ct >= con_frame_num) {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;
        double time_con_i = time_con;
        lidar_buffer.push_back(ptr_con_i);
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }
  bound_lidar_queue_locked();
  if (scan_count < MAXN) {
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  }
  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);

  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  // publish_count ++;

  msg->header.stamp = get_ros_time(
    get_time_sec(msg_in->header.stamp) - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

  double timestamp = get_time_sec(msg->header.stamp);
  // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

  if (timestamp < last_timestamp_imu) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "imu timestamp rolled back; reset buffers");
    clear_sensor_buffers_locked();
    last_timestamp_lidar = -1.0;
    flg_reset = true;
  } else if (last_timestamp_imu > 0.0 && timestamp - last_timestamp_imu > max_imu_time_gap) {
    imu_gap_detected = true;
    RCLCPP_WARN(
      rclcpp::get_logger("li_initialization"), "IMU gap %.3f s exceeds %.3f s",
      timestamp - last_timestamp_imu, max_imu_time_gap);
  }
  imu_deque.emplace_back(msg);
  while (imu_deque.size() > static_cast<std::size_t>(max_imu_queue_size)) {
    imu_deque.pop_front();
    ++dropped_imu_messages;
  }
  last_timestamp_imu = timestamp;
  sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup & meas)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  {
    if (!imu_en) {
      if (!lidar_buffer.empty()) {
        if (!lidar_pushed) {
          meas.lidar = lidar_buffer.front();
          meas.lidar_beg_time = time_buffer.front();
          lose_lid = false;
          if (meas.lidar->points.size() < static_cast<std::size_t>(min_scan_points)) {
            std::cout << "lose lidar" << '\n';
            lose_lid = true;
            ++dropped_lidar_frames;
          } else {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt : meas.lidar->points) {
              if (pt.curvature > end_time) {
                end_time = pt.curvature;
              }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            const double scan_duration = lidar_end_time - meas.lidar_beg_time;
            if (!std::isfinite(scan_duration) || scan_duration < 0.0 || scan_duration > max_scan_duration) {
              lose_lid = true;
              ++dropped_lidar_frames;
            }
          }
          lidar_pushed = true;
        }

        time_buffer.pop_front();
        lidar_buffer.pop_front();
        lidar_pushed = false;
        if (!lose_lid) {
          return true;
        } else {
          return false;
        }
      }
      return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty()) {
      return false;
    }
    /*** push a lidar scan ***/
    if (!lidar_pushed) {
      lose_lid = false;
      meas.lidar = lidar_buffer.front();
      meas.lidar_beg_time = time_buffer.front();
      if (meas.lidar->points.size() < static_cast<std::size_t>(min_scan_points)) {
        std::cout << "lose lidar" << '\n';
        lose_lid = true;
        ++dropped_lidar_frames;
        // lidar_buffer.pop_front();
        // time_buffer.pop_front();
        // return false;
      } else {
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt : meas.lidar->points) {
          if (pt.curvature > end_time) {
            end_time = pt.curvature;
          }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        // std::cout << "check time lidar:" << end_time << '\n';
        meas.lidar_last_time = lidar_end_time;
        const double scan_duration = lidar_end_time - meas.lidar_beg_time;
        if (!std::isfinite(scan_duration) || scan_duration < 0.0 || scan_duration > max_scan_duration) {
          RCLCPP_WARN(
            rclcpp::get_logger("li_initialization"),
            "reject LiDAR scan: points=%zu duration=%.4f s", meas.lidar->points.size(),
            scan_duration);
          lose_lid = true;
          ++dropped_lidar_frames;
        }
      }
      lidar_pushed = true;
    }

    if (lose_lid) {
      lidar_buffer.pop_front();
      time_buffer.pop_front();
      lidar_pushed = false;
      imu_pushed = false;
      return false;
    }

    if (last_timestamp_imu < lidar_end_time) {
      return false;
    }

    if (!lose_lid && !imu_pushed) {
      /*** push imu data, and pop from imu buffer ***/
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        imu_next = *(imu_deque.front());
        meas.imu.clear();
        while (imu_time < lidar_end_time) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);  // can be changed
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
  }
}
