#include <sdu_controllers/kinematics/forward_kinematics.hpp>

using namespace sdu_controllers;
using namespace sdu_controllers::kinematics;

ForwardKinematics::ForwardKinematics(const std::vector<ForwardKinematics::JointType>& jointType) : jointType_(jointType)
{
}

ForwardKinematics::ForwardKinematics(const std::vector<bool>& is_joint_revolute) : jointType_(is_joint_revolute.size())
{
  for (size_t i = 0; i < is_joint_revolute.size(); ++i)
  {
    jointType_[i] = is_joint_revolute[i] ? ForwardKinematics::REVOLUTE : ForwardKinematics::PRISMATIC;
  }
}

const std::vector<ForwardKinematics::JointType>& ForwardKinematics::get_joint_types() const
{
  return jointType_;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> ForwardKinematics::geometric_jacobian(const Eigen::VectorXd& q) const
{
  std::vector<Eigen::Matrix4d> T_chain = forward_kinematics_all(q);

  Eigen::Vector3d z_im1, o_im1, o_n;
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, jointType_.size());

  // Get position of end effector
  o_n = T_chain.back().template block<3, 1>(0, 3);

  // Loop through joints
  for (int i = 0; i < jointType_.size(); i++)
  {
    Eigen::Matrix4d T = T_chain[i];

    // Get z_{i-1}, o_{i-1}
    // z_{i-1} is the unit vector along the z-axis of the previous joint
    // o_{i-1} is the position of the center of the previous joint frame
    if (i > 0)
    {
      z_im1 = T_chain[i - 1].block<3, 1>(0, 2);
      o_im1 = T_chain[i - 1].block<3, 1>(0, 3);
    }
    else
    {
      // Base Position
      z_im1 = Eigen::Vector3d(0, 0, 1);
      o_im1 = Eigen::Vector3d(0, 0, 0);
    }

    // Assign the appropriate blocks of the Jacobian
    // These formulas are from Spong.
    if (jointType_[i] == ForwardKinematics::REVOLUTE)
    {
      // Revolute joint
      J.template block<3, 1>(0, i) = z_im1.cross(o_n - o_im1);
      J.template block<3, 1>(3, i) = z_im1;
    }
    else if (jointType_[i] == ForwardKinematics::PRISMATIC)
    {
      // Prismatic joint
      J.template block<3, 1>(0, i) = z_im1;
      J.template block<3, 1>(3, i).fill(0);
    }
    else
    {
      throw std::runtime_error("ForwardKinematics::geometric_jacobian: Unknown joint type");
    }
  }

  return J;
}

size_t ForwardKinematics::get_dof() const
{
  return jointType_.size();
}