#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>

constexpr double pi = 3.14159265358979323846;
using namespace Eigen;

namespace sdu_controllers::models
{
  URRobotModel::URRobotModel(RobotType robot_type) : RobotModel(), ur_robot_(robot_type)
  {
    VectorXd q_low(dof_);
    VectorXd q_high(dof_);
    VectorXd dq_low(dof_);
    VectorXd dq_high(dof_);
    VectorXd ddq_low(dof_);
    VectorXd ddq_high(dof_);
    VectorXd torque_low(dof_);
    VectorXd torque_high(dof_);

    if (robot_type == UR5e)
    {
      q_low << -2 * pi, -2 * pi, -2 * pi, -2 * pi, -2 * pi, -2 * pi;
      q_high << 2 * pi, 2 * pi, 2 * pi, 2 * pi, 2 * pi, 2 * pi;
      dq_low << -pi, -pi, -pi, -pi, -pi, -pi;
      dq_high << pi, pi, pi, pi, pi, pi;
      ddq_low << -40.0, -40.0, -40.0, -40.0, -40.0, -40.0;
      ddq_high << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0;
      torque_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      torque_high << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0;
    }
    else if (robot_type == UR3e)
    {
      q_low << -2 * pi, -2 * pi, -2 * pi, -2 * pi, -2 * pi, -1000;
      q_high << 2 * pi, 2 * pi, 2 * pi, 2 * pi, 2 * pi, 1000;
      dq_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      dq_high << pi, pi, pi, 2 * pi, 2 * pi, 2 * pi;
      ddq_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      ddq_high << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0;
      torque_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      torque_high << 56.0, 56.0, 28.0, 12.0, 12.0, 12.0;
    }

    joint_pos_bounds_ = { q_low, q_high};
    joint_vel_bounds_ = { dq_low, dq_high};
    joint_acc_bounds_ = { ddq_low, ddq_high};
    joint_torque_bounds_ = {torque_low, torque_high};
  }

  MatrixXd URRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    return ur_robot_.inertia(q);
  }

  MatrixXd URRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    return ur_robot_.coriolis(q, qd);
  }

  MatrixXd URRobotModel::get_gravity(const VectorXd& q)
  {
    return ur_robot_.gravity(q);
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_pos_bounds() const
  {
    return joint_pos_bounds_;
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_vel_bounds() const
  {
    return joint_vel_bounds_;
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_acc_bounds() const
  {
    return joint_acc_bounds_;
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_torque_bounds() const
  {
    return joint_torque_bounds_;
  }

  uint16_t URRobotModel::get_dof() const
  {
    return dof_;
  }

}  // namespace sdu_controllers::models
