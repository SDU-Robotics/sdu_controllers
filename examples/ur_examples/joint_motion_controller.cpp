#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/pd_controller.hpp>
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
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::URRobotModel>(URRobot::RobotType::UR5e);
  double freq = 500.0;
  double dt = 1.0 / freq;
  double Kp_value = 100.0;
  double Kd_value = 2 * sqrt(Kp_value);
  double N_value = 1;
  uint16_t ROBOT_DOF = robot_model->get_dof();
  VectorXd Kp_vec = VectorXd::Ones(ROBOT_DOF) * Kp_value;
  VectorXd Kd_vec = VectorXd::Ones(ROBOT_DOF) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(ROBOT_DOF) * N_value;

  controllers::PDController pd_controller(Kp_vec.asDiagonal(), Kd_vec.asDiagonal(), N_vec.asDiagonal());
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  math::ForwardDynamics fwd_dyn(robot_model);

  VectorXd q_d(ROBOT_DOF);
  VectorXd dq_d(ROBOT_DOF);
  VectorXd ddq_d(ROBOT_DOF);

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  q << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Read input trajectory from file
  std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file("../../examples/data/joint_trajectory_safe.csv");

  // Offline safety verification of the input trajectory.
  //  - checks joint position, velocity and acceleration limits.
  safety::SafetyVerifier safety_verifier(robot_model);
  bool is_trajectory_safe = safety_verifier.verify_trajectory_safety(input_trajectory);
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

      std::cout << "q_d: " << q_d << std::endl;

      // Add noise to q and dq
      VectorXd q_meas = q;
      VectorXd dq_meas = dq;
      add_noise_to_vector(q_meas, 0.0, 0.001);
      add_noise_to_vector(dq_meas, 0.0, 0.001);

      // Controller
      VectorXd u_ff = ddq_d; // acceleration as feedforward.
      // VectorXd u_ff = robot_model->get_gravity(q_meas); // feedforward with gravity compensation.
      pd_controller.step(q_d, dq_d, u_ff, q_meas, dq_meas);
      VectorXd y = pd_controller.get_output();
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
      csv_writer << eigen_to_std_vector(q);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    output_filestream.close();
  }
  else
  {
    std::cerr << "input trajectory is not safe!" << std::endl;
  }
}
