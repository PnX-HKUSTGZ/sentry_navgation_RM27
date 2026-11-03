#include <fstream>
#include <iomanip>
#include "startup_scenarios.hpp"
#include "IMU_Processing.h"

int main(int argc, char ** argv)
{
  const std::string filename = argc>1 ? argv[1] : "/tmp/pointlio_startup_comparison.csv";
  const int repetitions = argc>2 ? std::stoi(argv[2]) : 3;
  // Optional ZLIO-format YAML overrides for reproducible parameter sweeps.
  const auto overrides = argc > 3 ? std::optional<zlio::Config>(zlio::loadConfig(argv[3])) : std::nullopt;
  const std::string scenario_filter = argc > 4 ? argv[4] : "all";
  const std::string method_filter = argc > 5 ? argv[5] : "all";
  if ((scenario_filter != "all" && scenario_filter != "static") ||
      (method_filter != "all" && method_filter != "joint" && method_filter != "legacy")) {
    throw std::invalid_argument("expected scenario all/static and method all/joint/legacy");
  }
  std::ofstream csv(filename);
  csv << "scenario,repeat,method,success,sensor_seconds,compute_ms,tilt_error_deg,body_velocity_error_mps,gyro_bias_error_radps,accel_bias_error_mps2,map_shape_rmse_m,reason\n";
  csv << std::setprecision(10);
  for (const auto & scenario : startup_test::scenarios()) {
    if (scenario_filter == "static" &&
        (scenario.moving || scenario.plane_only)) {continue;}
    for (int repeat = 1; repeat <= repetitions; ++repeat) {
      for (std::string method : {"legacy","joint"}) {
        if (method_filter != "all" && method != method_filter) {continue;}
        auto config = startup_test::config(scenario);
        if (overrides) {
          config.solver.initialization = overrides->initialization;
          config.solver.initial_accel_bias = overrides->initial_accel_bias;
          config.solver.initial_accel_bias_std = overrides->initial_accel_bias_std;
        }
        point_lio::JointStartup joint(config);
        ImuProcess legacy;
        legacy.gravity_ = Eigen::Vector3d(0,0,-9.81);
        bool success = false;
        double elapsed = 0, cpu_ms = 0;
        zlio::State estimate;
        std::vector<zlio::MapPoint> map;
        std::string reason;
        for (int frame=0; frame<48 && !success; ++frame) {
          auto p = startup_test::packet(scenario,frame,repeat);
          const auto start=std::chrono::steady_clock::now();
          if (method=="joint") {
            auto result=joint.add(p.scan,p.begin,p.end,p.imus);
            reason=joint.reason();
            if (result) {
              success=true;
              estimate=result->solution.estimate.state;
              map=std::move(result->solution.map);
            }
          } else {
            MeasureGroup m;
            for (std::size_t j=0;j+1<p.imus.size();++j) {
              const auto & sample=p.imus[j];
              auto imu=std::make_shared<sensor_msgs::msg::Imu>();
              imu->angular_velocity.x=sample.angular_velocity.x();
              imu->angular_velocity.y=sample.angular_velocity.y();
              imu->angular_velocity.z=sample.angular_velocity.z();
              imu->linear_acceleration.x=sample.acceleration.x();
              imu->linear_acceleration.y=sample.acceleration.y();
              imu->linear_acceleration.z=sample.acceleration.z();
              m.imu.push_back(imu);
            }
            PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
            legacy.Process(m,cloud);
            if (!legacy.imu_need_init_) {
              Eigen::Vector3d gravity=-9.81*legacy.mean_acc.normalized();
              Eigen::Matrix3d rotation;
              legacy.Set_init(gravity,rotation);
              estimate.pose.rotation=Eigen::Quaterniond(rotation);
              estimate.time=p.end;
              for (const auto & point:p.scan) {
                map.push_back({estimate.pose.apply(scenario.extrinsic.apply(point.position)),
                  Eigen::Matrix3d::Zero()});
              }
              success=true;
              reason="sample_count_100";
            }
          }
          cpu_ms += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
          elapsed = zlio::seconds(p.end)-startup_test::epoch;
        }
        csv << scenario.name << ',' << repeat << ',' << method << ',' << success << ','
            << elapsed << ',' << cpu_ms << ',';
        if (success) {
          const auto actual=startup_test::truth(scenario,elapsed);
          csv << startup_test::tiltError(estimate,actual) << ','
              << startup_test::speedError(estimate,actual) << ','
              << (estimate.gyro_bias-scenario.gyro_bias).norm() << ','
              << (estimate.accel_bias-scenario.accel_bias).norm() << ','
              << startup_test::mapRmse(map,estimate,actual) << ',';
        } else {csv << ",,,,,";}
        std::replace(reason.begin(),reason.end(),',',';');
        csv << reason << '\n'; csv.flush();
      }
    }
  }
}
