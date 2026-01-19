#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/pid_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <vector>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

int main()
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters

  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobotModel::RobotType::ur5e);
  double freq = 500.0;
  double dt = 1.0 / freq;
  double Kp_value = 1000.0;
  double Ki_value = 100.0;
  double Kd_value = 2 * sqrt(Kp_value) * 0;
  double N_value = 1;
  uint16_t ROBOT_DOF = robot_model->get_dof();
  VectorXd Kp_vec = VectorXd::Ones(ROBOT_DOF) * Kp_value;
  VectorXd Ki_vec = VectorXd::Ones(ROBOT_DOF) * Ki_value;
  VectorXd Kd_vec = VectorXd::Ones(ROBOT_DOF) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(ROBOT_DOF) * N_value;

  VectorXd u_max(ROBOT_DOF);
  // UR5e max and min torque see https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
  u_max << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0;
  VectorXd u_min = -u_max;

  controllers::PIDController pid_controller(Kp_vec.asDiagonal(), Ki_vec.asDiagonal(),
    Kd_vec.asDiagonal(), N_vec.asDiagonal(), dt, u_min, u_max);
  //math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  //math::ForwardDynamics fwd_dyn(robot_model);

  VectorXd q_d(ROBOT_DOF);
  VectorXd dq_d(ROBOT_DOF);
  VectorXd ddq_d(ROBOT_DOF);

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  q << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Vector<double, 6> he = VectorXd::Zero(6);

  // Read input trajectory from file
  std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file(utils::data_path("joint_trajectory_safe.csv"));

  // Offline safety verification of the input trajectory.
  //  - checks joint position, velocity and acceleration limits.
  safety::SafetyVerifier safety_verifier(robot_model);
  bool is_trajectory_safe = safety_verifier.verify_trajectory_safety(input_trajectory);
  size_t j = 0.0;
  if (is_trajectory_safe)
  {
    // Control loop
    for (const std::vector<double>& trajectory_point : input_trajectory)
    {
      // Desired
      for (Index i = 0; i < q_d.size(); i++)
      {
        q_d[i] = trajectory_point[i];
        dq_d[i] = trajectory_point[i+ROBOT_DOF];
        ddq_d[i] = trajectory_point[i+(2*ROBOT_DOF)];
      }

      // Add noise to q and dq
      VectorXd q_meas = q;
      VectorXd dq_meas = dq;
      //add_noise_to_vector(q_meas, 0.0, 0.001);
      //add_noise_to_vector(dq_meas, 0.0, 0.001);

      // Controller
      VectorXd u_ff = ddq_d; // acceleration as feedforward.
      // VectorXd u_ff = robot_model->get_gravity(q_meas); // feedforward with gravity compensation.
      pid_controller.step(q_d, dq_d, u_ff, q_meas, dq_meas);
      VectorXd y = pid_controller.get_output();
      std::cout << "y: " << y << std::endl;

      VectorXd tau = robot_model->inverse_dynamics(q_meas, dq_meas, y, he);
      std::cout << "tau: " << tau << std::endl;

      // Simulation
      VectorXd ddq = robot_model->forward_dynamics(q, dq, tau);
      // integrate to get velocity
      dq += ddq * dt;
      // integrate to get position
      q += dq * dt;

      std::cout << "q:" << q << std::endl;
      VectorXd temp(1 + q.size());
      temp << j * dt, q;
      csv_writer << eigen_to_std_vector(temp);
      j++;
    }
    output_filestream.close();
  }
  else
  {
    std::cerr << "input trajectory is not safe!" << std::endl;
  }
}
