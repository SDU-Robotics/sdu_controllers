#pragma once
#ifndef SDU_CONTROLLERS_UTILITY_HPP
#define SDU_CONTROLLERS_UTILITY_HPP

#include <vector>
#include <random>
#include <Eigen/Dense>
#include <sdu_controllers/utils/csv.hpp>

namespace sdu_controllers::utils
{
  inline std::vector<std::vector<double>> get_trajectory_from_file(const std::string& filepath)
  {
    csv::CSVReader reader(filepath);
    std::vector<std::vector<double>> trajectory;
    for (const auto& row : reader)
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
  bool is_within_bounds(const T& value, const T& low, const T& high)
  {
    return !(value < low) && (value < high);
  }

  template<typename T>
  Eigen::Matrix<T, Eigen::Dynamic, 1> std_vector_to_eigen(const std::vector<T>& vec)
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

  // Template function to convert an Eigen::Matrix (vector) to a std::vector
  template<typename T>
  std::vector<T> eigen_to_std_vector(const Eigen::Matrix<T, Eigen::Dynamic, 1>& eigen_vec)
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

  inline void add_noise_to_vector(Eigen::VectorXd &vec, const double mean=0.0, const double std_dev=0.2)
  {
    // Define random generator with Gaussian distribution
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution dist(mean, std_dev);
    for (Eigen::Index i = 0; i < vec.size(); i++)
      vec[i] += dist(generator);
  }

}  // namespace sdu_controllers::utils

#endif  // SDU_CONTROLLERS_UTILITY_HPP
