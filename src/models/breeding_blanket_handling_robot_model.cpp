#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel() : RobotModel()
  {
    VectorXd q_low(dof_);
    VectorXd q_high(dof_);
    VectorXd dq_low(dof_);
    VectorXd dq_high(dof_);
    VectorXd ddq_low(dof_);
    VectorXd ddq_high(dof_);
    VectorXd torque_low(dof_);
    VectorXd torque_high(dof_);

    constexpr double pi = 3.14159265358979323846;

    q_low << -25, -pi/2, 0.935, -2 * pi, -2 * pi, -2 * pi, -5;
    q_high << 8, pi/2, 4.66, 2 * pi, 2 * pi, 2 * pi, 5;
    dq_low << -pi, -pi, -pi, -pi, -pi, -pi, -pi;
    dq_high << pi, pi, pi, pi, pi, pi, pi;
    ddq_low << -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0;
    ddq_high << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0;
    torque_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    torque_high << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0, 28.0;

    joint_pos_bounds_ = { q_low, q_high};
    joint_vel_bounds_ = { dq_low, dq_high};
    joint_acc_bounds_ = { ddq_low, ddq_high};
    joint_torque_bounds_ = {torque_low, torque_high};

    a_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    d_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    alpha_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    is_joint_revolute_ = {false, false, true, true, true, true, true};
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_gravity(const VectorXd& q)
  {
    VectorXd v = VectorXd::Zero(ROBOT_DOF);
    return v;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_jacobian(const VectorXd& q)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_jacobian_dot(const VectorXd& q, const VectorXd& dq)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_pos_bounds()
  {
    return joint_pos_bounds_;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_vel_bounds()
  {
    return joint_vel_bounds_;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_acc_bounds()
  {
    return joint_acc_bounds_;
  }

  std::pair<VectorXd, VectorXd> BreedingBlanketHandlingRobotModel::get_joint_torque_bounds()
  {
    return joint_torque_bounds_;
  }

  uint16_t BreedingBlanketHandlingRobotModel::get_dof() const
  {
    return dof_;
  }

  std::vector<double> BreedingBlanketHandlingRobotModel::get_a()
  {
    return a_;
  }
  std::vector<double> BreedingBlanketHandlingRobotModel::get_d()
  {
    return d_;
  }
  std::vector<double> BreedingBlanketHandlingRobotModel::get_alpha()
  {
    return alpha_;
  }

}  // namespace sdu_controllers::math
