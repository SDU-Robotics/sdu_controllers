#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/operational_space_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

int main()
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output_cartesian.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobot::RobotType::UR5e);
  double freq = 500.0;
  double dt = 1.0 / freq;

  double Kp_pos_value = 16250.0;
  double Kp_orient_value = 16250.0;
  double Kd_pos_value = 200.0;
  double Kd_orient_value = 3.0;
  double N_value = 1;
  uint16_t ROBOT_DOF = robot_model->get_dof();

  VectorXd Kp_pos_vec = VectorXd::Ones(3) * Kp_pos_value;
  VectorXd Kp_orient_vec = VectorXd::Ones(3) * Kp_orient_value;
  VectorXd Kd_pos_vec = VectorXd::Ones(3) * Kd_pos_value;
  VectorXd Kd_orient_vec = VectorXd::Ones(3) * Kd_orient_value;
  VectorXd N_vec = VectorXd::Ones(6) * N_value;

  MatrixXd Kp = MatrixXd::Zero(6, 6);
  Kp.setIdentity();
  Kp.block<3, 3>(0,0) = Kp_pos_vec.asDiagonal();
  Kp.block<3, 3>(3,3) = Kp_orient_vec.asDiagonal();
  MatrixXd Kd = MatrixXd::Zero(6, 6);
  Kd.setIdentity();
  Kd.block<3, 3>(0,0) = Kd_pos_vec.asDiagonal();
  Kd.block<3, 3>(3,3) = Kd_orient_vec.asDiagonal();

  controllers::OperationalSpaceController osc_controller(Kp, Kd, robot_model);
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  math::ForwardDynamics fwd_dyn(robot_model);

  VectorXd x_d(6);
  VectorXd dx_d(6);
  VectorXd ddx_d(6);

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  q << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Read input trajectory from file
  std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file("../../examples/data/cartesian_trajectory_safe.csv");

  // Control loop
  for (size_t j=0; j<input_trajectory.size(); j++)  // (const std::vector<double>& trajectory_point : input_trajectory)
  {
    // Desired
    for (Index i = 0; i < x_d.size(); i++)
    {
      x_d[i] = input_trajectory[j][i];
      dx_d[i] = input_trajectory[j][i+6];
      ddx_d[i] = input_trajectory[j][i+12];
    }

    std::cout << "x_d: " << x_d << std::endl;

    // Add noise to q and dq
    VectorXd q_meas = q;
    VectorXd dq_meas = dq;
    //add_noise_to_vector(q_meas, 0.0, 0.001);
    //add_noise_to_vector(dq_meas, 0.0, 0.001);

    // Controller
    osc_controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas);
    VectorXd y = osc_controller.get_output();
    std::cout << "y: " << y << std::endl;
    VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q_meas, dq_meas);
    std::cout << "tau: " << tau << std::endl;

    // Simulation
    VectorXd ddq = fwd_dyn.forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;


    std::cout << "q:" << q << std::endl;
    MatrixXd T = kinematics::forward_kinematics(q, robot_model);
    VectorXd pos = T.block<3, 1>(0, 3);
    std::cout << "pos:" << pos << std::endl;
    Matrix3d rot_mat = T.block<3,3>(0, 0);
    Vector3d rpy_zyz = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    std::cout << "rpy_zyz:" << rpy_zyz << std::endl;
    VectorXd temp(q.size()+pos.size()+rpy_zyz.size());
    temp << q, pos, rpy_zyz;
    csv_writer << eigen_to_std_vector(temp);
  }
  output_filestream.close();
}
