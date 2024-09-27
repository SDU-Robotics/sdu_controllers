#include <Eigen/Dense>
#include <iostream>
#include <sdu_controllers/controllers/pd_controller.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/robot_model.hpp>

using namespace Eigen;
using namespace sdu_controllers;

int main()
{
  // TODO: Load trajectory from csv file.

  // Desired
  VectorXd q_d(6);
  q_d << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  VectorXd dq_d(6);
  dq_d << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  VectorXd ddq_d(6);
  ddq_d << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

  // TODO: Get measured from simulated robot using ur_rtde
  
  // Measured
  VectorXd q(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  VectorXd dq(6);
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


  auto robot_model = std::make_shared<models::URRobotModel>(UR5e);
  double Kp_value = 0.1;
  double Kd_value = sqrt(Kp_value);
  double N_value = 1;
  VectorXd Kp_vec = VectorXd::Ones(robot_model->get_dof()) * Kp_value;
  VectorXd Kd_vec = VectorXd::Ones(robot_model->get_dof()) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(robot_model->get_dof()) * N_value;

  controllers::PDController pd_controller(Kp_vec.asDiagonal(), Kd_vec.asDiagonal(),N_vec.asDiagonal());
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  pd_controller.step(q_d, dq_d, ddq_d, q, dq);
  VectorXd y = pd_controller.get_output();
  std::cout << "y: " << y << std::endl;
  VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q, dq);
  std::cout << "tau: " << tau << std::endl;
}
