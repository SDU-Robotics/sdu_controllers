#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <sdu_controllers/math/rnea.hpp>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

int main()
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("eurofusion_states.csv");
  auto csv_writer = make_csv_writer(output_filestream);
  csv_writer << std::vector<std::string>({"timestamp", "q0", "q1", "q2", "q3", "q4", "q5", "q6",
    "tau0", "tau1", "tau2", "tau3", "tau4", "tau5", "tau6"});

  // Initialize robot model and parameters
  // auto robot_model = std::make_shared<models::URRobotModel>(URRobot::RobotType::UR5e);
  auto robot_model = std::make_shared<models::BreedingBlanketHandlingRobotModel>();
  double freq = 1000.0;
  double dt = 1.0 / freq;

  uint16_t ROBOT_DOF = robot_model->get_dof();

  // math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  // math::ForwardDynamics fwd_dyn(robot_model);

  math::RecursiveNewtonEuler rnea(robot_model);
  Eigen::Vector3d z0;
  z0 << 0, 0, -1;
  rnea.set_z0(z0);

  VectorXd q_d(ROBOT_DOF);
  VectorXd dq_d(ROBOT_DOF);
  VectorXd ddq_d(ROBOT_DOF);

  // VectorXd q(ROBOT_DOF);
  // VectorXd dq(ROBOT_DOF);
  // VectorXd ddq(ROBOT_DOF);
  // // q << 1, 1, 1, 1, 1, 1, 1; // 1.5707, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  // q << 0,0,0,0,0,3.1415,0;
  // dq << q;
  // ddq << q;
  // // dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  //
  Vector<double, 6> he;
  he.setZero();
  //
  // Eigen::VectorXd tau = rnea.inverse_dynamics(q, dq, ddq, he);
  // std::cout << "tau\n" << tau << std::endl;
  Eigen::VectorXd tau;

  // Load trajectory
  std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file("../../examples/data/breeder_trajectory_interpolated.csv");
  // std::vector<double> xData = {0., 5, 10, 12, 20, 25, 30, 35, 45, 50, 80};
  // std::vector<std::vector<double>> yData = {
  //   {0.0027, 2.115, 2.685, 2.685, 2.7168, 2.8036, 1.5, 0.7636, -4.768, -7.5, -24.5273},
  //   {0.000400, 0.140000, 0.140000, 0.140000, 0.000000, -0.054100, -0.140000, -0.140000, -0.127822, -0.122000, -0.083300},
  //   {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.300000, 0.000000},
  //   {0.000000, -0.006000, -0.006000, -0.006000, -0.024100, -0.030200, -0.030200, 0.000000, 0.000000, 0.000000, 0.000000},
  //   {-0.000400, 0.308000, 0.308000, 0.308000, -0.186100, -0.746200, -0.746200, 1.842600, 1.418971, 1.220000, 0.041900},
  //   {-0.062000, -0.313700, -0.313700, -0.313700, 0.202800, 0.980000, 0.980000, -1.764600, -1.352137, -1.150000, -0.020600},
  //   {0.000000, -0.000900, -0.000900, -0.000900, -0.123000, -0.155000, -0.155000, -0.150500, -0.116510, 0.060000, 0.000000}
  // };

  // Loop
  int j = 0;
  for (const std::vector<double>& trajectory_point : input_trajectory)
  // for (int j = 0; j < 10/dt; ++j)
  {
    for (Index i = 0; i < q_d.size(); i++)
    {
      q_d[i] = trajectory_point[i];
      dq_d[i] = trajectory_point[i+ROBOT_DOF];
      ddq_d[i] = trajectory_point[i+(2*ROBOT_DOF)];

      // q_d[i] = sin(j * dt + double(i)/double(ROBOT_DOF));
      // dq_d[i] = cos(j * dt + double(i)/double(ROBOT_DOF));
      // ddq_d[i] = -sin(j * dt + double(i)/double(ROBOT_DOF));
    }

    tau << rnea.inverse_dynamics(q_d, dq_d, ddq_d, he);

    VectorXd temp(1 + q_d.size() + tau.size());
    temp << j * dt, q_d, tau;
    csv_writer << eigen_to_std_vector(temp);

    ++j;
  }

  //  // Control loop
  //  for (const std::vector<double>& trajectory_point : input_trajectory)
  //  {
  //    // Desired
  //    for (Index i = 0; i < q_d.size(); i++)
  //    {
  //      q_d[i] = trajectory_point[i];
  //      dq_d[i] = trajectory_point[i+ROBOT_DOF];
  //      ddq_d[i] = trajectory_point[i+(2*ROBOT_DOF)];
  //    }
  //
  //    std::cout << "q_d: " << q_d << std::endl;
  //
  //    // Add noise to q and dq
  //    VectorXd q_meas = q;
  //    VectorXd dq_meas = dq;
  //    // add_noise_to_vector(q_meas, 0.0, 0.001);
  //    // add_noise_to_vector(dq_meas, 0.0, 0.001);
  //
  //    // Controller
  //    VectorXd u_ff = ddq_d; // acceleration as feedforward.
  //    // VectorXd u_ff = robot_model->get_gravity(q_meas); // feedforward with gravity compensation.
  //    pd_controller.step(q_d, dq_d, u_ff, q_meas, dq_meas);
  //    VectorXd y = pd_controller.get_output();
  //    std::cout << "y: " << y << std::endl;
  //    // VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q_meas, dq_meas);
  //    VectorXd tau = rnea.inverse_dynamics(q_meas, dq_meas, y, he);
  //    std::cout << "tau: " << tau << std::endl;
  //
  //    // Simulation
  //    VectorXd ddq = fwd_dyn.forward_dynamics(q, dq, tau);
  //    // integrate to get velocity
  //    dq += ddq * dt;
  //    // integrate to get position
  //    q += dq * dt;
  //
  //    std::cout << "q:" << q << std::endl;
  //    csv_writer << eigen_to_std_vector(q);
  //    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  //  }
  //
  // }
  // else
  // {
  //  std::cerr << "input trajectory is not safe!" << std::endl;
  // }

  output_filestream.close();
}
