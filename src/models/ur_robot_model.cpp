#include <iostream>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>

constexpr double pi = 3.14159265358979323846;
using namespace Eigen;

namespace sdu_controllers::models
{
  URRobotModel::URRobotModel() : ur_robot_(URRobot::RobotType::UR5e)
  {
    VectorXd q_low(dof_);
    VectorXd q_high(dof_);
    VectorXd dq_low(dof_);
    VectorXd dq_high(dof_);
    VectorXd ddq_low(dof_);
    VectorXd ddq_high(dof_);
    VectorXd torque_low(dof_);
    VectorXd torque_high(dof_);

    q_low << -2 * pi, -2 * pi, -2 * pi, -2 * pi, -2 * pi, -2 * pi;
    q_high << 2 * pi, 2 * pi, 2 * pi, 2 * pi, 2 * pi, 2 * pi;
    dq_low << -pi, -pi, -pi, -pi, -pi, -pi;
    dq_high << pi, pi, pi, pi, pi, pi;
    ddq_low << -40.0, -40.0, -40.0, -40.0, -40.0, -40.0;
    ddq_high << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0;
    torque_low << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    torque_high << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0;

    joint_pos_bounds_ = { q_low, q_high };
    joint_vel_bounds_ = { dq_low, dq_high };
    joint_acc_bounds_ = { ddq_low, ddq_high };
    joint_torque_bounds_ = { torque_low, torque_high };
  }

  URRobotModel::URRobotModel(URRobot::RobotType robot_type) : ur_robot_(robot_type)
  {
    VectorXd q_low(dof_);
    VectorXd q_high(dof_);
    VectorXd dq_low(dof_);
    VectorXd dq_high(dof_);
    VectorXd ddq_low(dof_);
    VectorXd ddq_high(dof_);
    VectorXd torque_low(dof_);
    VectorXd torque_high(dof_);

    if (robot_type == URRobot::RobotType::UR5e)
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
    else if (robot_type == URRobot::RobotType::UR3e)
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
    std::vector<double> grav = ur_robot_.gravity(q);
    // std::cout << "grav: ";
    // for (int i=0; i<grav.size(); ++i)
    //   std::cout << grav[i] << ' ';
    // std::cout << std::endl;
    return utils::std_vector_to_eigen(grav);
    //return ur_robot_.gravity(q);
  }

  MatrixXd URRobotModel::get_jacobian(const VectorXd& q)
  {
    return ur_robot_.jacobian(q);
  }

  MatrixXd URRobotModel::get_jacobian_dot(const VectorXd& q, const VectorXd& dq)
  {
    return ur_robot_.jacobian_dot(q, dq);
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_pos_bounds()
  {
    return joint_pos_bounds_;
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_vel_bounds()
  {
    return joint_vel_bounds_;
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_acc_bounds()
  {
    return joint_acc_bounds_;
  }

  std::pair<VectorXd, VectorXd> URRobotModel::get_joint_torque_bounds()
  {
    return joint_torque_bounds_;
  }

  uint16_t URRobotModel::get_dof() const
  {
    return dof_;
  }

  std::vector<double> URRobotModel::get_a()
  {
    return ur_robot_.get_a();
  }
  std::vector<double> URRobotModel::get_d()
  {
    return ur_robot_.get_d();
  }
  std::vector<double> URRobotModel::get_alpha()
  {
    return ur_robot_.get_alpha();
  }

}  // namespace sdu_controllers::models
