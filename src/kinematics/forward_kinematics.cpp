#include <sdu_controllers/kinematics/forward_kinematics.hpp>

using namespace sdu_controllers;

std::vector<Eigen::Matrix4d> kinematics::forward_kinematics_all(
    const Eigen::VectorXd& q,
    const std::vector<kinematics::DHParam>& dh_parameters)
{
  //  Initialize data
  Eigen::Matrix4d jTj, bTc = Eigen::Matrix4d::Identity();

  std::vector<Eigen::Matrix4d> T_chain(dh_parameters.size());
  // Iterate over joints
  for (size_t k = 0; k < dh_parameters.size(); k++)
  {
    // Get the joint parameters
    DHParam dh = dh_parameters[k];

    // Apply the joint value
    if (false)  // if prismatic
      dh.d += q(k);
    else
      dh.theta += q(k);

    double cos_alpha = cos(dh.alpha);
    double sin_alpha = sin(dh.alpha);
    double cos_theta = cos(dh.theta);
    double sin_theta = sin(dh.theta);

    // Calculate the transformation matrix
    jTj << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, dh.a * cos_theta, sin_theta, cos_theta * cos_alpha,
        -cos_theta * sin_alpha, dh.a * sin_theta, 0, sin_alpha, cos_alpha, dh.d, 0, 0, 0, 1;

    bTc *= jTj;
    T_chain[k] = bTc;
  }  // End of joint for loop
  return T_chain;
}
