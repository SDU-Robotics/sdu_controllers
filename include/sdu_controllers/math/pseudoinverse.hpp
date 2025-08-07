//
// Created by eld on 30-05-25.
//
// Taken from https://gist.github.com/javidcf/25066cf85e71105d57b6

#pragma once
#ifndef SDU_CONTROLLERS_PSEUDOINVERSE_HPP
#define SDU_CONTROLLERS_PSEUDOINVERSE_HPP

namespace sdu_controllers::math
{
  template <class MatT>
  Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
  pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
  {
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singular_values = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singular_values_inv(mat.cols(), mat.rows());
    singular_values_inv.setZero();
    for (unsigned int i = 0; i < singular_values.size(); ++i) {
      if (singular_values(i) > tolerance)
      {
        singular_values_inv(i, i) = Scalar{1} / singular_values(i);
      }
      else
      {
        singular_values_inv(i, i) = Scalar{0};
      }
    }
    return svd.matrixV() * singular_values_inv * svd.matrixU().adjoint();
  }
}

#endif //SDU_CONTROLLERS_PSEUDOINVERSE_HPP
