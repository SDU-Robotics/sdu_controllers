#pragma once
#ifndef SDU_CONTROLLERS_UTILITY_HPP
#define SDU_CONTROLLERS_UTILITY_HPP

#include <Eigen/Dense>
#include <random>
#include <sdu_controllers/math/pose.hpp>
#include <sdu_controllers/utils/csv.hpp>
#include <vector>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace sdu_controllers::utils
{
  /**
   * @brief Generates a trapezoidal Cartesian trajectory between two poses.
   *
   * This function calculates a trajectory with a trapezoidal velocity profile,
   * moving from a start pose to a target pose over a specified time.
   *
   * @param start_pose The starting pose.
   * @param target_pose The target pose.
   * @param total_time The total time for the trajectory.
   * @param acceleration_time The time spent accelerating and decelerating.
   * @param frequency The frequency of the control (used for sampling intermediate points).

   * @return A vector of poses representing the trajectory.
   */
  inline std::vector<math::Pose> generate_trapezoidal_trajectory(
      const math::Pose &start_pose,
      const math::Pose &target_pose,
      double total_time,
      double acceleration_time,
      double frequency = 1000.0)
  {
    std::vector<math::Pose> trajectory;

    if (total_time <= 0 || acceleration_time <= 0 || acceleration_time * 2 >= total_time)
    {
      std::cerr << "Invalid trajectory parameters." << std::endl;
      return trajectory;  // Return empty trajectory on error.
    }

    double deceleration_time = acceleration_time;
    double constant_velocity_time = total_time - 2 * acceleration_time;

    // Calculate velocity and acceleration
    Eigen::Vector3d start_position = start_pose.get_position();
    Eigen::Quaterniond start_orientation = start_pose.get_orientation();
    Eigen::Vector3d target_position = target_pose.get_position();
    Eigen::Quaterniond target_orientation = target_pose.get_orientation();

    Eigen::Vector3d delta_position = target_position - start_position;

    double acceleration = delta_position.norm() / (acceleration_time * (total_time - acceleration_time));
    Eigen::Vector3d acceleration_vector = delta_position.normalized() * acceleration;

    Eigen::Vector3d velocity = acceleration_vector * acceleration_time;

    // Generate trajectory points
    double dt = 1.0 / frequency;  // Time step
    for (double t = 0; t <= total_time; t += dt)
    {
      Eigen::Vector3d current_position;
      Eigen::Quaterniond current_orientation;
      double slerp_fraction;

      if (t < acceleration_time)
      {
        // Acceleration phase
        current_position = start_position + 0.5 * acceleration_vector * t * t;
        slerp_fraction = t / acceleration_time;
        current_orientation = start_orientation.slerp(slerp_fraction, target_orientation);
      }
      else if (t < acceleration_time + constant_velocity_time)
      {
        // Constant velocity phase
        double constant_time = t - acceleration_time;
        current_position =
            start_position + acceleration_vector * acceleration_time * (0.5 * acceleration_time) + velocity * constant_time;
        slerp_fraction = 1.0;
        current_orientation = start_orientation.slerp(slerp_fraction, target_orientation);
      }
      else
      {
        // Deceleration phase
        double deceleration_start_time = acceleration_time + constant_velocity_time;
        double deceleration_elapsed_time = t - deceleration_start_time;

        current_position = target_position - 0.5 * acceleration_vector * std::pow((total_time - t), 2);
        slerp_fraction = (total_time - t) / acceleration_time;
        current_orientation = start_orientation.slerp(1 - slerp_fraction, target_orientation);
      }
      trajectory.emplace_back(current_position, current_orientation);
    }
    return trajectory;
  }

  inline std::vector<std::vector<double>> get_trajectory_from_file(const std::string &filepath)
  {
    csv::CSVReader reader(filepath);
    std::vector<std::vector<double>> trajectory;
    for (const auto &row : reader)
    {
      std::vector<double> trajectory_point(row.size());
      for (size_t i = 0; i < row.size(); i++)
      {
        trajectory_point[i] = row[i].get<double>();
      }
      trajectory.push_back(trajectory_point);
    }
    return trajectory;
  }

  template<typename T>
  bool is_within_bounds(const T &value, const T &low, const T &high)
  {
    return (value < high) && (value > low);
  }

  template<typename T>
  Eigen::Matrix<T, Eigen::Dynamic, 1> std_vector_to_eigen(const std::vector<T> &vec)
  {
    // Create an Eigen vector with the same size as std::vector
    Eigen::Matrix<T, Eigen::Dynamic, 1> eigen_vec(vec.size());

    // Copy elements from std::vector to Eigen vector
    for (std::size_t i = 0; i < vec.size(); ++i)
    {
      eigen_vec[i] = vec[i];
    }

    return eigen_vec;
  }

  /**
   * @brief Parse a YAML sequence node of numbers directly into an Eigen::VectorXd.
   * @returns Eigen::VectorXd with the numbers or std::nullopt on invalid node or conversion failure.
   */
  inline std::optional<Eigen::VectorXd> yaml_node_to_eigen_vector(const YAML::Node &node)
  {
    if (!node || !node.IsSequence())
      return std::nullopt;

    Eigen::VectorXd v(static_cast<int>(node.size()));
    try
    {
      for (std::size_t i = 0; i < node.size(); ++i)
      {
        v(static_cast<int>(i)) = node[i].as<double>();
      }
      return v;
    }
    catch (const YAML::BadConversion &)
    {
      return std::nullopt;
    }
  }

  /**
   * @brief Converts a std::array of doubles to an Eigen::Vector<double, N>
   *
   * This function takes a std::array of doubles and converts it to an Eigen::Vector<double, N>.
   * The size of the output vector will match the size of the input array.
   *
   * @tparam N The size of the input std::array
   * @param arr The input std::array of doubles to convert
   * @return Eigen::Vector<double, N> The resulting Eigen vector
   *
   * @example
   * std::array<double, 3> my_array = {1.0, 2.0, 3.0};
   * Eigen::Vector<double, N> my_vec = std_array_to_eigen_vector(my_array);
   */
  template<std::size_t N>
  Eigen::Vector<double, N> std_array_to_eigen_vector(const std::array<double, N> &arr)
  {
    Eigen::Vector<double, N> vec;
    for (std::size_t i = 0; i < N; ++i)
    {
      vec(i) = arr[i];
    }
    return vec;
  }

  // Template function to convert an Eigen::Matrix (vector) to a std::vector
  template<typename T>
  std::vector<T> eigen_to_std_vector(const Eigen::Matrix<T, Eigen::Dynamic, 1> &eigen_vec)
  {
    // Create a std::vector with the same size as the Eigen vector
    std::vector<T> std_vec(eigen_vec.size());

    // Copy elements from Eigen vector to std::vector
    for (std::size_t i = 0; i < eigen_vec.size(); ++i)
    {
      std_vec[i] = eigen_vec[i];
    }

    return std_vec;
  }

  inline void add_noise_to_vector(Eigen::VectorXd &vec, const double mean = 0.0, const double std_dev = 0.2)
  {
    // Define random generator with Gaussian distribution
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution dist(mean, std_dev);
    for (Eigen::Index i = 0; i < vec.size(); i++)
      vec[i] += dist(generator);
  }

  static Eigen::Affine3d pos_rotvec_to_T(const Eigen::Vector3d &position, const Eigen::AngleAxisd &rotation)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = position;
    T.linear() = rotation.toRotationMatrix();
    return T;
  }

  static Eigen::Affine3d pos_rotmat_to_T(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = position;
    T.linear() = rotation;
    return T;
  }

  static Eigen::Affine3d pos_quat_to_T(const Eigen::VectorXd &pose)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = pose.block<3, 1>(0, 0);
    T.linear() = Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).toRotationMatrix();
    return T;
  }

  static Eigen::Affine3d stdvec_to_T(const std::vector<double> &pose)
  {
    Eigen::Vector3d position(pose[0], pose[1], pose[2]);

    Eigen::Vector3d compact_eaa(pose[3], pose[4], pose[5]);
    double angle = compact_eaa.norm();

    Eigen::Affine3d T = pos_rotvec_to_T(position, Eigen::AngleAxisd(angle, compact_eaa.normalized()));
    return T;
  }

  static std::vector<double> T_to_stdvec(const Eigen::Affine3d &T)
  {
    std::vector<double> pose;
    for (double p : T.translation())
    {
      pose.push_back(p);
    }

    Eigen::AngleAxisd eaa(T.linear());
    Eigen::Vector3d compact_eaa(eaa.angle() * eaa.axis());
    for (double p : compact_eaa)
    {
      pose.push_back(p);
    }

    return pose;
  }

  static std::string data_path(const std::string &rel)
  {
  #ifdef PROJECT_SOURCE_DIR
    return std::string(PROJECT_SOURCE_DIR) + "/data/" + rel;
  #else
    return "data/" + rel;
  #endif
  }

  static std::string project_path(const std::string &rel)
  {
  #ifdef PROJECT_SOURCE_DIR
    return std::string(PROJECT_SOURCE_DIR) + "/" + rel;
  #else
    return rel;
  #endif
  }

}  // namespace sdu_controllers::utils

#endif  // SDU_CONTROLLERS_UTILITY_HPP
