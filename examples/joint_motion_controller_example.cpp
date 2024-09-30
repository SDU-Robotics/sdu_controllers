#include <sdu_controllers/controllers/pd_controller.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace ur_rtde;
using namespace sdu_controllers;

int main()
{
  std::vector<std::vector<std::string>> trajectory = utils::read_csv("../../examples/data/trajectory.csv");

  RTDEControlInterface rtde_c("192.168.56.101");
  RTDEReceiveInterface rtde_r("192.168.56.101");

  auto robot_model = std::make_shared<models::URRobotModel>(UR5e);
  double Kp_value = 0.1;
  double Kd_value = sqrt(Kp_value);
  double N_value = 1;
  VectorXd Kp_vec = VectorXd::Ones(robot_model->get_dof()) * Kp_value;
  VectorXd Kd_vec = VectorXd::Ones(robot_model->get_dof()) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(robot_model->get_dof()) * N_value;

  controllers::PDController pd_controller(Kp_vec.asDiagonal(), Kd_vec.asDiagonal(), N_vec.asDiagonal());
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);

  // Move simulated robot to initial position.
  rtde_c.moveJ({0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0});

  // Control loop
  for (const auto& row : trajectory)
  {
    // Measured
    std::vector q_measured = rtde_r.getActualQ();
    std::vector qd_measured = rtde_r.getActualQd();
    VectorXd q = Eigen::VectorXd::Map(&q_measured[0], q_measured.size());
    VectorXd dq = Eigen::VectorXd::Map(&qd_measured[0], qd_measured.size());

    // Desired
    VectorXd q_d(6);
    VectorXd dq_d(6);
    VectorXd ddq_d(6);

    for (size_t i = 0; i < 6; i++)
      q_d << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
    for (size_t i = 6; i < 12; i++)
      dq_d << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    for (size_t i = 12; i < 18; i++)
      ddq_d << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    pd_controller.step(q_d, dq_d, ddq_d, q, dq);
    VectorXd y = pd_controller.get_output();
    std::cout << "y: " << y << std::endl;
    VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q, dq);
    std::cout << "tau: " << tau << std::endl;
  }
}
