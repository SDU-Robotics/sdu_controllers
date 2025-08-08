#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/math.hpp>

using namespace sdu_controllers;

Eigen::Matrix<double, 6, Eigen::Dynamic> math::jacobian(
    const std::vector<Eigen::Matrix4d>& T_chain,
    const std::vector<kinematics::DHParam>& dh_parameters)
{
  // Initialize some variables
  Eigen::Vector3d z_im1, o_im1, o_n;
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, dh_parameters.size());

  // Get position of end effector
  o_n = T_chain.back().template block<3, 1>(0, 3);

  // Loop through joints
  for (int i = 0; i < dh_parameters.size(); i++)
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
    if (dh_parameters[i].is_joint_revolute)
    {
      // Revolute joint
      J.template block<3, 1>(0, i) = z_im1.cross(o_n - o_im1);
      J.template block<3, 1>(3, i) = z_im1;
    }
    else
    {
      // Prismatic joint
      J.template block<3, 1>(0, i) = z_im1;
      J.template block<3, 1>(3, i).fill(0);
    }
  }

  return J;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> math::jacobian(
    Eigen::VectorXd q,
    const std::vector<kinematics::DHParam>& dh_parameters)
{
  // Get the forward kinematics chain
  std::vector<Eigen::Matrix4d> T_chain = kinematics::forward_kinematics_all(q, dh_parameters);
  // Calculate the Jacobian
  return math::jacobian(T_chain, dh_parameters);
}