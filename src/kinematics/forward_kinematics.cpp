#include <sdu_controllers/kinematics/forward_kinematics.hpp>

using namespace sdu_controllers;
using namespace sdu_controllers::kinematics;

ForwardKinematics::ForwardKinematics(const std::vector<ForwardKinematics::JointType>& joint_type) : joint_type_(joint_type)
{
}

const std::vector<ForwardKinematics::JointType>& ForwardKinematics::get_joint_types() const
{
  return joint_type_;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> ForwardKinematics::geometric_jacobian(const Eigen::VectorXd& q) const
{
  std::vector<Eigen::Matrix4d> T_chain = forward_kinematics_all(q);

  return geometric_jacobian(T_chain);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> ForwardKinematics::geometric_jacobian(
    const std::vector<Eigen::Matrix4d>& fk_matrices) const
{
  Eigen::Vector3d z_im1, o_im1, o_n;
  const size_t n_dof = get_dof();
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n_dof);

  // Get position of TCP (end-effector with TCP offset applied)
  o_n = (fk_matrices.back() * tcp_transform_).template block<3, 1>(0, 3);

  // Loop through all links; only fill Jacobian columns for actuated joints
  int col = 0;
  for (int i = 0; i < static_cast<int>(joint_type_.size()); i++)
  {
    if (i > 0)
    {
      z_im1 = fk_matrices[i - 1].block<3, 1>(0, 2);
      o_im1 = fk_matrices[i - 1].block<3, 1>(0, 3);
    }
    else
    {
      // Base Position
      z_im1 = Eigen::Vector3d(0, 0, 1);
      o_im1 = Eigen::Vector3d(0, 0, 0);
    }

    if (joint_type_[i] == ForwardKinematics::FIXED)
    {
      // Fixed joints contribute no column to the Jacobian
      continue;
    }
    else if (joint_type_[i] == ForwardKinematics::REVOLUTE)
    {
      J.template block<3, 1>(0, col) = z_im1.cross(o_n - o_im1);
      J.template block<3, 1>(3, col) = z_im1;
    }
    else if (joint_type_[i] == ForwardKinematics::PRISMATIC)
    {
      J.template block<3, 1>(0, col) = z_im1;
      J.template block<3, 1>(3, col).fill(0);
    }
    else
    {
      throw std::runtime_error("ForwardKinematics::geometric_jacobian: Unknown joint type");
    }
    ++col;
  }

  return J;
}

Eigen::Matrix4d ForwardKinematics::forward_kinematics(const std::vector<double>& q) const
{
  Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  return forward_kinematics(q_eigen);
}

std::vector<Eigen::Matrix4d> ForwardKinematics::forward_kinematics_all(const std::vector<double>& q) const
{
  Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  return forward_kinematics_all(q_eigen);
}

size_t ForwardKinematics::get_dof() const
{
  size_t count = 0;
  for (const auto& jt : joint_type_)
    if (jt != ForwardKinematics::FIXED)
    ++count;
  return count;
}

size_t ForwardKinematics::get_num_links() const
{
  return joint_type_.size();
}

void ForwardKinematics::set_tcp(const Eigen::Matrix4d& tcp_transform)
{
  tcp_transform_ = tcp_transform;
}

const Eigen::Matrix4d& ForwardKinematics::get_tcp() const
{
  return tcp_transform_;
}

Eigen::Matrix4d ForwardKinematics::get_tcp_pose(const Eigen::VectorXd& q) const
{
  return forward_kinematics(q) * tcp_transform_;
}

Eigen::Matrix4d ForwardKinematics::get_tcp_pose(const std::vector<double>& q) const
{
  Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  return get_tcp_pose(q_eigen);
}
