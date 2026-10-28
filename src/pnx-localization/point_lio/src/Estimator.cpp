// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"

PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
std::vector<int> time_seq;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
std::vector<V3D> pbody_list;
std::vector<PointVector> Nearest_Points;
std::shared_ptr<IVoxType> ivox_ = nullptr;  // localmap in ivox
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
std::vector<std::uint8_t> point_selected_surf;
std::vector<M3D> crossmat_list;
ActiveLidarPointVector active_lidar_points;
point_lio::FrameDiagnostics frame_diagnostics;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
esekfom::esekf<state_input, 24, input_ikfom> kf_input;
esekfom::esekf<state_output, 30, input_ikfom> kf_output;
input_ikfom input_in;
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
double G_m_s2 = 9.81;

Eigen::Matrix<double, 24, 24> process_noise_cov_input()
{
  Eigen::Matrix<double, 24, 24> cov;
  cov.setZero();
  cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
  cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
  cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  // MTK::get_cov<process_noise_input>::type cov = MTK::get_cov<process_noise_input>::type::Zero();
  // MTK::setDiagonal<process_noise_input, vect3, 0>(cov, &process_noise_input::ng, gyr_cov_input);// 0.03
  // MTK::setDiagonal<process_noise_input, vect3, 3>(cov, &process_noise_input::na, acc_cov_input); // *dt 0.01 0.01 * dt * dt 0.05
  // MTK::setDiagonal<process_noise_input, vect3, 6>(cov, &process_noise_input::nbg, b_gyr_cov); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
  // MTK::setDiagonal<process_noise_input, vect3, 9>(cov, &process_noise_input::nba, b_acc_cov);   //0.001 0.05 0.0001/out 0.01
  return cov;
}

Eigen::Matrix<double, 30, 30> process_noise_cov_output()
{
  Eigen::Matrix<double, 30, 30> cov;
  cov.setZero();
  cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
  cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
  cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
  cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

Eigen::Matrix<double, 24, 1> get_f_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);  // .normalized()
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = omega[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 30, 1> get_f_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
  vect3 a_inertial = s.rot * s.acc;  // .normalized()
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = s.omg[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 24, 24> df_dx_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(acc_);  // .normalized().toRotationMatrix()
  cov.template block<3, 3>(12, 18) = -s.rot;                  //.normalized().toRotationMatrix();
  // Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  // Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  // s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();  // grav_matrix;
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

Eigen::Matrix<double, 30, 30> df_dx_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(s.acc);  // .normalized().toRotationMatrix()
  cov.template block<3, 3>(12, 18) = s.rot;                    //.normalized().toRotationMatrix();
  // Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  // Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  // s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();  // grav_matrix;
  cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
  return cov;
}

namespace
{
struct WeightedMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  const ActiveLidarPoint * active_point = nullptr;
  V3D point_imu = V3D::Zero();
  V3D point_world = V3D::Zero();
  V3D normal = V3D::Zero();
  double offset = 0.0;
  double coefficient = 1.0;
  double residual = 0.0;
  double innovation_variance = 1.0;
  double effective_weight = 1.0;
  std::size_t scale_index = 0;
};

using WeightedMeasurementVector =
  std::vector<WeightedMeasurement, Eigen::aligned_allocator<WeightedMeasurement>>;

template <typename State>
V3D resolvePointImu(const ActiveLidarPoint & active, const State & state)
{
  if (active.reanchored || !extrinsic_est_en) {
    return active.point_imu;
  }
  return state.offset_R_L_I * active.point_lidar + state.offset_T_L_I;
}

template <typename State>
void buildLidarMeasurement(
  State & state, const Eigen::Matrix3d & cov_p, const Eigen::Matrix3d & cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  WeightedMeasurementVector measurements;
  const int query_count = spatial_adaptive_config.enabled ?
    spatial_adaptive_config.neighbors.back() : NUM_MATCH_POINTS;
  const double query_range = spatial_adaptive_config.enabled ?
    std::min(max_neighbor_distance, spatial_adaptive_config.max_query_ranges.back()) :
    max_neighbor_distance;

  for (const auto & active : active_lidar_points) {
    if (active.source_index >= feats_down_world->size() ||
        active.source_index >= Nearest_Points.size()) {
      continue;
    }

    const V3D point_imu = resolvePointImu(active, state);
    const V3D point_world = state.rot * point_imu + state.pos;
    PointType & point_world_msg = feats_down_world->points[active.source_index];
    point_world_msg.x = point_world.x();
    point_world_msg.y = point_world.y();
    point_world_msg.z = point_world.z();
    point_world_msg.intensity = feats_down_body->points[active.source_index].intensity;

    auto & points_near = Nearest_Points[active.source_index];
    ivox_->GetClosestPoint(point_world_msg, points_near, query_count, query_range);
    std::sort(
      points_near.begin(), points_near.end(), [&point_world_msg](const PointType & lhs, const PointType & rhs) {
        return calc_dist<double>(point_world_msg, lhs) < calc_dist<double>(point_world_msg, rhs);
      });
    point_selected_surf[active.source_index] = 0;

    if (!spatial_adaptive_config.enabled) {
      if (points_near.size() < NUM_MATCH_POINTS ||
          calc_dist<double>(point_world_msg, points_near[NUM_MATCH_POINTS - 1]) >
            max_neighbor_distance * max_neighbor_distance) {
        continue;
      }
      VF(4) plane;
      plane.setZero();
      if (!esti_plane(plane, points_near, plane_thr)) {
        continue;
      }
      const V3D normal = plane.head<3>().cast<double>();
      const double residual = normal.dot(point_world) + static_cast<double>(plane(3));
      if (point_imu.norm() <= match_s * residual * residual) {
        continue;
      }

      WeightedMeasurement measurement;
      measurement.active_point = &active;
      measurement.point_imu = point_imu;
      measurement.point_world = point_world;
      measurement.normal = normal;
      measurement.offset = plane(3);
      measurement.residual = residual;
      measurement.innovation_variance = laser_point_cov;
      measurements.emplace_back(measurement);
      point_selected_surf[active.source_index] = 1;
      frame_diagnostics.associated_points++;
      continue;
    }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> neighbor_vectors;
    neighbor_vectors.reserve(points_near.size());
    for (const auto & neighbor : points_near) {
      neighbor_vectors.emplace_back(neighbor.x, neighbor.y, neighbor.z);
    }

    point_lio::PlaneModelVector models;
    for (std::size_t scale = 0; scale < spatial_adaptive_config.neighbors.size(); ++scale) {
      const int count = spatial_adaptive_config.neighbors[scale];
      if (count < 3 || points_near.size() < static_cast<std::size_t>(count)) {
        continue;
      }
      const double range = spatial_adaptive_config.max_query_ranges[scale];
      if (calc_dist<double>(point_world_msg, points_near[count - 1]) > range * range) {
        continue;
      }

      point_lio::PlaneModel model;
      if (!point_lio::fitPlane(
            neighbor_vectors, static_cast<std::size_t>(count),
            spatial_adaptive_config.max_curvature,
            spatial_adaptive_config.max_plane_residual, model)) {
        continue;
      }
      model.scale_index = scale;
      model.residual = model.normal.dot(point_world) + model.offset;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_imu);
      const V3D rotated_normal = state.rot.transpose() * model.normal;
      const V3D rotation_jacobian = point_crossmat * rotated_normal;
      const double state_variance =
        model.normal.dot(cov_p * model.normal) +
        rotation_jacobian.dot(cov_R * rotation_jacobian);
      model.surface_variance =
        std::max(model.surface_variance, spatial_adaptive_config.min_surface_variance);
      model.innovation_variance =
        std::max(laser_point_cov + model.surface_variance + state_variance, 1.0e-9);
      models.emplace_back(model);
    }

    point_lio::computeResponsibilities(
      models, spatial_adaptive_config.outlier_probability,
      spatial_adaptive_config.normalized_residual_gate,
      spatial_adaptive_config.student_t_dof,
      spatial_adaptive_config.min_responsibility);

    bool point_accepted = false;
    for (const auto & model : models) {
      if (model.responsibility <= 0.0 || model.robust_weight <= 0.0) {
        continue;
      }
      const double measurement_variance =
        std::max(laser_point_cov + model.surface_variance, 1.0e-9);
      const double effective_weight = model.responsibility * model.robust_weight;
      const double coefficient = std::min(
        std::sqrt(effective_weight * laser_point_cov / measurement_variance),
        spatial_adaptive_config.max_information_scale);
      if (!std::isfinite(coefficient) || coefficient <= 0.0) {
        continue;
      }

      WeightedMeasurement measurement;
      measurement.active_point = &active;
      measurement.point_imu = point_imu;
      measurement.point_world = point_world;
      measurement.normal = model.normal;
      measurement.offset = model.offset;
      measurement.coefficient = coefficient;
      measurement.residual = model.residual;
      measurement.innovation_variance = model.innovation_variance;
      measurement.effective_weight = effective_weight;
      measurement.scale_index = model.scale_index;
      measurements.emplace_back(measurement);
      frame_diagnostics.scale_responsibility[model.scale_index] += model.responsibility;
      frame_diagnostics.effective_points += effective_weight;
      point_accepted = true;
    }
    if (point_accepted) {
      point_selected_surf[active.source_index] = 1;
      frame_diagnostics.associated_points++;
    }
  }

  if (measurements.empty()) {
    ekfom_data.valid = false;
    return;
  }

  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x = Eigen::MatrixXd::Zero(measurements.size(), 12);
  ekfom_data.z.resize(measurements.size());
  for (std::size_t row = 0; row < measurements.size(); ++row) {
    const auto & measurement = measurements[row];
    const V3D weighted_normal = measurement.coefficient * measurement.normal;
    M3D point_imu_crossmat;
    point_imu_crossmat << SKEW_SYM_MATRX(measurement.point_imu);
    const V3D C(state.rot.transpose() * weighted_normal);
    const V3D A(point_imu_crossmat * C);

    if (extrinsic_est_en && !measurement.active_point->reanchored) {
      M3D point_lidar_crossmat;
      point_lidar_crossmat << SKEW_SYM_MATRX(measurement.active_point->point_lidar);
      const V3D B(point_lidar_crossmat * state.offset_R_L_I.transpose() * C);
      ekfom_data.h_x.block<1, 12>(row, 0) << VEC_FROM_ARRAY(weighted_normal),
        VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(row, 0) << VEC_FROM_ARRAY(weighted_normal),
        VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    ekfom_data.z(row) = -measurement.coefficient * measurement.residual;
    frame_diagnostics.residual_abs_sum += std::abs(measurement.residual);
    frame_diagnostics.nis_sum += measurement.residual * measurement.residual /
                                 std::max(measurement.innovation_variance, 1.0e-9);
  }

  if (spatial_adaptive_config.enabled && measurements.size() >= 6) {
    const Eigen::Matrix<double, 6, 6> raw_information =
      ekfom_data.h_x.leftCols(6).transpose() * ekfom_data.h_x.leftCols(6);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(raw_information);
    if (solver.info() == Eigen::Success) {
      const double largest = std::max(solver.eigenvalues().maxCoeff(), 1.0e-12);
      Eigen::Matrix<double, 6, 6> observable = Eigen::Matrix<double, 6, 6>::Zero();
      for (int axis = 0; axis < 6; ++axis) {
        if (solver.eigenvalues()(axis) >= largest * spatial_adaptive_config.degeneracy_eigen_ratio) {
          observable(axis, axis) = 1.0;
        }
      }
      const Eigen::Matrix<double, 6, 6> projection =
        solver.eigenvectors() * observable * solver.eigenvectors().transpose();
      ekfom_data.h_x.leftCols(6) = ekfom_data.h_x.leftCols(6) * projection;
    }
  }

  frame_diagnostics.effective_rows += measurements.size();
  frame_diagnostics.pose_information.noalias() +=
    ekfom_data.h_x.leftCols(6).transpose() * ekfom_data.h_x.leftCols(6) /
    std::max(laser_point_cov, 1.0e-9);
  if (!spatial_adaptive_config.enabled) {
    frame_diagnostics.effective_points += static_cast<double>(measurements.size());
  }
  effct_feat_num += static_cast<int>(measurements.size());
}
}  // namespace

void h_model_input(
  state_input & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  buildLidarMeasurement(s, cov_p, cov_R, ekfom_data);
}

void h_model_output(
  state_output & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  buildLidarMeasurement(s, cov_p, cov_R, ekfom_data);
}

void h_model_IMU_output(state_output & s, esekfom::dyn_share_modified<double> & ekfom_data)
{
  std::memset(ekfom_data.satu_check, false, 6);
  ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;
  ekfom_data.z_IMU.block<3, 1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
  ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov,
    imu_meas_acc_cov, imu_meas_acc_cov;
  if (check_satu) {
    if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[0] = true;
      frame_diagnostics.imu_saturated = true;
      ekfom_data.z_IMU(0) = 0.0;
    }

    if (fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[1] = true;
      frame_diagnostics.imu_saturated = true;
      ekfom_data.z_IMU(1) = 0.0;
    }

    if (fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[2] = true;
      frame_diagnostics.imu_saturated = true;
      ekfom_data.z_IMU(2) = 0.0;
    }

    if (fabs(acc_avr(0)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[3] = true;
      frame_diagnostics.imu_saturated = true;
      ekfom_data.z_IMU(3) = 0.0;
    }

    if (fabs(acc_avr(1)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[4] = true;
      frame_diagnostics.imu_saturated = true;
      ekfom_data.z_IMU(4) = 0.0;
    }

    if (fabs(acc_avr(2)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[5] = true;
      frame_diagnostics.imu_saturated = true;
      ekfom_data.z_IMU(5) = 0.0;
    }
  }
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
  V3D p_body(pi->x, pi->y, pi->z);

  V3D p_global;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_global =
        kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) +
        kf_output.x_.pos;
    } else {
      p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) +
                 kf_input.x_.pos;
    }
  } else {
    if (!use_imu_as_input) {
      p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_output.x_.pos;  // .normalized()
    } else {
      p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_input.x_.pos;  // .normalized()
    }
  }

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}
