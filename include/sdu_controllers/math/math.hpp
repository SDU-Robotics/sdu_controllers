#pragma once
#ifndef SDU_CONTROLLERS_MATH_HPP
#define SDU_CONTROLLERS_MATH_HPP

#include <Eigen/Dense>

namespace sdu_controllers::math
{
  template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0],
            -vec[1], vec[0], 0.0)
        .finished();
  }

  Eigen::Quaterniond exp(const Eigen::Quaterniond &quat)
  {
    using ::std::cos;
    using ::std::sin;

    double theta = quat.vec().norm();
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    Eigen::Quaterniond q;

    if (theta > double(0))
    {
      q.vec() = sin_theta * quat.vec() / theta;
    }
    else
    {
      q.vec().setZero();
    }

    q.w() = cos_theta;

    return q;
  }

} // namespace sdu_controllers::math


#endif  // SDU_CONTROLLERS_MATH