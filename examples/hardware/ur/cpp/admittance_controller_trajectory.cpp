#include <chrono>
#include <csignal>
#include <iostream>
#include <sdu_controllers/controllers/admittance_controller_cartesian.hpp>
#include <sdu_controllers/hal/ur_robot.hpp>
#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <thread>

using namespace Eigen;
using namespace std::chrono;
using namespace sdu_controllers::hal;
using namespace sdu_controllers::math;
using namespace sdu_controllers::utils;
using namespace sdu_controllers::controllers;

// Interrupt flag
bool flag_loop = true;
void raise_flag(int)
{
  flag_loop = false;
}

Vector3d
get_circle_target(const Vector3d& pose, const double timestep, const double radius = 0.075, const double freq = 0.35)
{
  Vector3d circle_target;
  circle_target[0] = pose[0] + radius * cos(2 * M_PI * freq * timestep);
  circle_target[1] = pose[1] + radius * sin(2 * M_PI * freq * timestep);
  circle_target[2] = pose[2];
  return circle_target;
}

int main(int argc, char* argv[])
{
  double frequency = 500.0;
  double dt = 1. / frequency;

  // Initialize admittance control
  VectorXd u;
  AdmittanceControllerCartesian adm_controller(frequency);
  adm_controller.set_mass_matrix_position(Vector3d(22.5, 22.5, 22.5).asDiagonal());
  adm_controller.set_stiffness_matrix_position(Vector3d(54, 54, 54).asDiagonal());
  adm_controller.set_damping_matrix_position(Vector3d(200, 200, 200).asDiagonal());

  adm_controller.set_mass_matrix_orientation(Vector3d(0.25, 0.25, 0.25).asDiagonal());
  adm_controller.set_stiffness_matrix_orientation(Vector3d(10, 10, 10).asDiagonal());
  adm_controller.set_damping_matrix_orientation(Vector3d(5, 5, 5).asDiagonal());

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  // Initialize UR Robot through HAL
  URRobot robot(robot_ip, frequency);

  robot.zero_ft_sensor();
  std::this_thread::sleep_for(200ms);

  Pose actual_tcp_pose = robot.get_cartesian_tcp_pose();
  Affine3d T_base_tcp = actual_tcp_pose.to_transform();  // stdvec_to_T(robot.getActualTCPPose());

  // Define tip
  Affine3d T_tcp_tip = Pose(Vector3d(0, 0, 0.194), Quaterniond::Identity()).to_transform();
  Affine3d T_tip_tcp = T_tcp_tip.inverse();
  Affine3d T_base_tip = T_base_tcp * T_tcp_tip;

  Vector3d pos_init = T_base_tip.translation();
  auto quat_init = Quaterniond(T_base_tip.linear());
  Vector4d quat_init_vec(quat_init.w(), quat_init.x(), quat_init.y(), quat_init.z());

  // Set target circle
  double counter = 0.0;
  Vector3d x_desired = get_circle_target(T_base_tip.translation(), counter);
  Affine3d T_base_tip_circle = pos_rotmat_to_T(x_desired, T_base_tip.linear());
  Affine3d T_base_tcp_circle = T_base_tip_circle * T_tip_tcp;
  Pose robot_pose(T_base_tcp_circle);  //   std::vector<double> robot_pose = T_to_stdvec(T_base_tcp_circle);
  robot.move_cartesian(robot_pose);

  signal(SIGINT, raise_flag);

  robot.set_control_mode(URRobot::ControlMode::CARTESIAN_POSE);
  robot.start_control();

  try
  {
    while (flag_loop)
    {
      // Start time
      steady_clock::time_point start_time = robot.init_period();

      // Get current position
      T_base_tcp = robot.get_cartesian_tcp_pose();
      Eigen::Vector<double, 6> ft = robot.get_tcp_forces();

      // Transform into compliant coordinate system (tip)
      T_base_tip = T_base_tcp * T_tcp_tip;

      // Get current force & torque
      Vector3d f_base(ft[0], ft[1], ft[2]);
      Vector3d mu_base(ft[3], ft[4], ft[5]);

      // Rotate forces from base frame into TCP
      Matrix3d R_tcp_base = T_base_tcp.linear().inverse();
      Vector3d f_tcp = R_tcp_base * f_base;
      Vector3d mu_tcp = R_tcp_base * mu_base;

      // use wrench transform to place the force torque in the tip.
      VectorXd ft_tip = wrench_trans(mu_tcp, f_tcp, T_tcp_tip);

      // Rotate forces back to base frame
      Vector3d f_base_tip = T_base_tip.linear() * ft_tip.block<3, 1>(3, 0);

      // Get circle target
      x_desired = get_circle_target(pos_init, counter);

      // Step controller
      adm_controller.step(f_base_tip, ft_tip.block<3, 1>(0, 0));;
      u = adm_controller.get_position_output(x_desired, quat_init_vec);

      // Rotate output from tip to TCP before sending it to the robot
      Affine3d T_base_tip_out = Pose(eigen_to_std_vector(u));
      Affine3d T_base_tcp_out = T_base_tip_out * T_tip_tcp;

      // Set control reference
      robot.set_cartesian_pose_ref(Pose(T_base_tcp_out));

      // Update the robot control
      robot.step();

      robot.wait_period(start_time);
      counter = counter + dt;
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
