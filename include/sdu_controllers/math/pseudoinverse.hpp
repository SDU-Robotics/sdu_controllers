//
// Created by eld on 30-05-25.
//
// Taken from https://gist.github.com/javidcf/25066cf85e71105d57b6

#ifndef PSEUDOINVERSE_HPP
#define PSEUDOINVERSE_HPP

namespace sdu_controllers::math
{
  template <class MatT>
  Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
  pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
  {
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
      if (singularValues(i) > tolerance)
      {
        singularValuesInv(i, i) = Scalar{1} / singularValues(i);
      }
      else
      {
        singularValuesInv(i, i) = Scalar{0};
      }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
  }
}

#endif //PSEUDOINVERSE_HPP
