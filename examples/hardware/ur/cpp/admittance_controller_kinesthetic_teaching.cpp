#include <signal.h>

#include <Eigen/Dense>
#include <chrono>
#include <sdu_controllers/controllers/admittance_controller_position.hpp>
#include <sdu_controllers/hal/ur_robot.hpp>
#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace Eigen;
using namespace std::chrono;
using namespace sdu_controllers::math;
using namespace sdu_controllers::utils;
using namespace sdu_controllers::hal;
using namespace sdu_controllers::controllers;

// Interrupt flag
bool flag_loop = true;
void raise_flag(int)
{
  flag_loop = false;
}

int main(int argc, char* argv[])
{
  double frequency = 500.0;
  double dt = 1. / frequency;

  // Initialize admittance control
  VectorXd u;
  AdmittanceControllerPosition adm_controller(frequency);

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  // Initialize UR Robot through HAL
  URRobot robot(robot_ip, frequency);

  std::this_thread::sleep_for(500ms);
  robot.zero_ft_sensor();
  std::this_thread::sleep_for(200ms);

  Pose actual_tcp_pose = robot.get_cartesian_tcp_pose();
  Affine3d T_base_tcp = actual_tcp_pose.to_transform();

  // Define tip
  Affine3d T_tcp_tip = Pose(Vector3d(0, 0, 0.05), Quaterniond::Identity()).to_transform();
  Affine3d T_tip_tcp = T_tcp_tip.inverse();
  Affine3d T_base_tip_init = T_base_tcp * T_tcp_tip;

  Vector3d pos_init = T_base_tip_init.translation();
  Quaterniond quat_init = Quaterniond(T_base_tip_init.rotation());
  Vector4d quat_init_vec(quat_init.w(), quat_init.x(), quat_init.y(), quat_init.z());

  adm_controller.set_mass_matrix_position(Vector3d(22.5, 22.5, 22.5).asDiagonal());
  adm_controller.set_stiffness_matrix_position(Vector3d(0, 0, 0).asDiagonal());
  adm_controller.set_damping_matrix_position(Vector3d(65, 65, 65).asDiagonal());

  adm_controller.set_mass_matrix_orientation(Vector3d(0.25, 0.25, 0.25).asDiagonal());
  adm_controller.set_stiffness_matrix_orientation(Vector3d(0, 0, 0).asDiagonal());
  adm_controller.set_damping_matrix_orientation(Vector3d(5, 5, 5).asDiagonal());

  // Uncomment this for the highly damped mode (stable on table)
  // adm_controller.set_damping_matrix_position(Vector3d(3250, 3250, 3250));
  // adm_controller.set_damping_matrix_orientation(Vector3d(25, 25, 25));

  signal(SIGINT, raise_flag);

  robot.set_control_mode(URRobot::ControlMode::CARTESIAN_POSE);
  robot.start_control();

  std::vector<double> robot_pose;

  try
  {
    while (flag_loop)
    {
      // Start time
      steady_clock::time_point start_time = robot.init_period();

      // Get current position
      T_base_tcp = robot.get_cartesian_tcp_pose();
      std::vector<double> ft = robot.get_tcp_forces();

      // Transform into compliant coordinate system (tip?)
      Affine3d T_base_tip = T_base_tcp * T_tcp_tip;

      // Get current force & torque
      Vector3d f_base(ft[0], ft[1], ft[2]);
      Vector3d mu_base(ft[3], ft[4], ft[5]);

      // Rotate forces from base frame into TCP
      Matrix3d R_tcp_base = T_base_tcp.rotation().inverse();
      Vector3d f_tcp = R_tcp_base * f_base;
      Vector3d mu_tcp = R_tcp_base * mu_base;

      // use wrench transform to place the force torque in the tip.
      VectorXd ft_tip = wrench_trans(mu_tcp, f_tcp, T_tcp_tip);

      // Rotate forces back to base frame
      Vector3d f_base_tip = T_base_tip.rotation() * ft_tip.block<3, 1>(3, 0);

      // Step controller
      adm_controller.step(f_base_tip, ft_tip.block<3, 1>(0, 0), pos_init, quat_init_vec);
      u = adm_controller.get_output();

      // Rotate output from tip to TCP before sending it to the robot
      Affine3d T_base_tip_out = Pose(eigen_to_std_vector(u));
      Affine3d T_base_tcp_out = T_base_tip_out * T_tip_tcp;

      // Set control reference
      robot.set_cartesian_pose_ref(Pose(T_base_tcp_out));

      // Update the robot control
      robot.step();

      robot.wait_period(start_time);
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  // Shutdown
  robot.stop_control();
  return 0;
}
