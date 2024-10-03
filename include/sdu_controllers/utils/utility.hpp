#pragma once
#ifndef SDU_CONTROLLERS_UTILITY_HPP
#define SDU_CONTROLLERS_UTILITY_HPP

#include <vector>
#include <random>
#include <Eigen/Dense>

namespace sdu_controllers::utils
{

  template<typename T>
  bool isWithinBounds(const T& value, const T& low, const T& high)
  {
    return !(value < low) && (value < high);
  }

  template<typename T>
  Eigen::Matrix<T, Eigen::Dynamic, 1> stdVectorToEigen(const std::vector<T>& vec)
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
  std::vector<T> eigenToStdVector(const Eigen::Matrix<T, Eigen::Dynamic, 1>& eigen_vec)
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

  inline void addNoiseToVector(Eigen::VectorXd &vec, const double mean=0.0, const double std_dev=0.2)
  {
    // Define random generator with Gaussian distribution
    std::default_random_engine generator;
    std::normal_distribution dist(mean, std_dev);
    for (Eigen::Index i = 0; i < vec.size(); i++)
      vec[i] += dist(generator);
  }

}  // namespace sdu_controllers::utils

#endif  // SDU_CONTROLLERS_UTILITY_HPP
