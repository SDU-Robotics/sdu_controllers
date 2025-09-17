#include <Eigen/Dense>
#include <signal.h>
#include <fstream>
#include <limits>
#include <iostream>
#include <sdu_controllers/controllers/pid_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/hal/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace csv;
using namespace std::chrono;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;
using namespace sdu_controllers::math;
using namespace sdu_controllers::hal;

constexpr double pi = 3.14159265358979323846;

// Interrupt flag
bool flag_loop = true;
void raise_flag(int)
{
  flag_loop = false;
}

Eigen::VectorXd clamp_array(const Eigen::VectorXd& value, const Eigen::VectorXd& clamp_value)
{
  Eigen::VectorXd ret = value;
  Eigen::VectorXd pdiff = value - clamp_value;
  Eigen::VectorXd mdiff = value + clamp_value;

  for (int j = 0; j < value.size(); ++j) {
      if (pdiff[j] > 0) {
          ret[j] = clamp_value[j];
      } else if (mdiff[j] < 0) {
          ret[j] = -clamp_value[j];
      }
  }
  return ret;
}


int main(int argc, char* argv[])
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);
  std::vector<std::string> header = {"t", "q0", "q1", "q2", "q3", "q4", "q5", "q_d0", "q_d1", "q_d2", "q_d3", "q_d4", "q_d5", "tau0", "tau1", "tau2", "tau3", "tau4", "tau5", "actual_current0", "actual_current1", "actual_current2", "actual_current3", "actual_current4", "actual_current5","target_current0", "target_current1", "target_current2", "target_current3", "target_current4", "target_current5"};
  csv_writer << header;

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  // Initialize UR Robot through HAL
  double freq = 500.0;
  double dt = 1.0 / freq;
  URRobot robot(robot_ip, freq);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobot::RobotType::UR5e);
  double N_value = 1;
  uint16_t ROBOT_DOF = robot_model->get_dof();
  VectorXd Kp_vec(ROBOT_DOF);
  Kp_vec << 2400.0, 1000.0, 6000.0, 20000.0, 70000.0, 1000000.0;
  VectorXd Ki_vec(ROBOT_DOF);
  Ki_vec << 8000.0, 2000.0, 3000.0, 8000.0, 10000.0, 130000.0;
  VectorXd Kd_vec(ROBOT_DOF);
  Kd_vec << 1*sqrt(Kp_vec[0]), 1*sqrt(Kp_vec[1]), 1*sqrt(Kp_vec[2]), 1*sqrt(Kp_vec[3]), 2*sqrt(Kp_vec[4]), 2*sqrt(Kp_vec[5]);
  VectorXd N_vec = VectorXd::Ones(ROBOT_DOF) * N_value;

  VectorXd u_max(ROBOT_DOF);
  // UR5e max and min torque see https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
  u_max << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0;
  VectorXd u_min = -u_max;

  controllers::PIDController pid_controller(Kp_vec.asDiagonal(), Ki_vec.asDiagonal(),
    Kd_vec.asDiagonal(), N_vec.asDiagonal(), dt, u_min, u_max);
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  math::ForwardDynamics fwd_dyn(robot_model);

  VectorXd q_d(ROBOT_DOF);
  VectorXd dq_d(ROBOT_DOF);
  VectorXd ddq_d(ROBOT_DOF);

  VectorXd q_init(ROBOT_DOF);
  q_init << -pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0.0;
  // Move robot to init position
  robot.move_joints(q_init);

  // Read input trajectory from file
  std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file("../../../../data/joint_trajectory_safe_new.csv");

  // Offline safety verification of the input trajectory.
  //  - checks joint position, velocity and acceleration limits.
  safety::SafetyVerifier safety_verifier(robot_model);
  bool is_trajectory_safe = safety_verifier.verify_trajectory_safety(input_trajectory);
  if (is_trajectory_safe)
  {
    signal(SIGINT, raise_flag);

    robot.set_control_mode(URRobot::ControlMode::TORQUE);
    robot.start_control();
    robot.step();

    int counter = 0;
    // Control loop
    for (const std::vector<double>& trajectory_point : input_trajectory)
    {
      // Start time
      steady_clock::time_point start_time = robot.init_period();

      // Desired
      for (Index i = 0; i < q_d.size(); i++)
      {
        q_d[i] = trajectory_point[i];
        dq_d[i] = trajectory_point[i+ROBOT_DOF];
        ddq_d[i] = trajectory_point[i+(2*ROBOT_DOF)];
      }
      std::cout << "q_d: " << q_d << std::endl;
      VectorXd q_meas = robot.get_joint_positions();
      VectorXd dq_meas = robot.get_joint_velocities();
      VectorXd actual_currents = robot.get_actual_joint_currents();
      VectorXd target_currents = robot.get_target_joint_currents();

      // Controller
      VectorXd u_ff = ddq_d; // acceleration as feedforward.
      //VectorXd u_ff = robot_model->get_gravity(q_meas); // feedforward with gravity compensation.
      pid_controller.step(q_d, dq_d, u_ff, q_meas, dq_meas);
      VectorXd y = pid_controller.get_output();
      //std::cout << "y: " << y << std::endl;
      VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q_meas, dq_meas);

      VectorXd tau_clamped = clamp_array(tau, u_max);
      std::cout << "tau_clamped: " << tau_clamped << std::endl;
      VectorXd temp(1 + q_meas.size() + q_d.size() + tau_clamped.size() + actual_currents.size() + target_currents.size());
      temp << counter * dt, q_meas, q_d, tau_clamped, actual_currents, target_currents;
      csv_writer << eigen_to_std_vector(temp);

      // Set joint torque reference
      robot.set_joint_torque_ref(tau_clamped);

      // Update the robot control
      robot.step();

      //std::cout << "q:" << robot.get_joint_positions() << std::endl;
      //csv_writer << eigen_to_std_vector(q);

      robot.wait_period(start_time);
      counter++;
    }
    output_filestream.close();
    robot.stop_control();
    // Move robot to current joint position to end torque mode.
    robot.move_joints(robot.get_joint_positions());
  }
  else
  {
    std::cerr << "input trajectory is not safe!" << std::endl;
  }
}
