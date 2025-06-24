#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sdu_controllers/hal/franka_robot.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace Eigen;
using namespace sdu_controllers::hal;
using namespace sdu_controllers::math;
using namespace sdu_controllers::utils;
using namespace std::chrono;

int main(int argc, char* argv[])
{
  double frequency = 1000.0;
  double dt = 1. / frequency;

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  try
  {
    // Initialize Franka Robot through HAL
    FrankaRobot robot(robot_ip, frequency);

    VectorXd q_d(7);
    q_d << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;

    std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Move to the desired joint-position in joint-space.
    double speed_factor = 0.1;
    robot.move_joints(q_d, speed_factor);
    std::cout << "Motion finished" << std::endl;

    // Get the measured joint position from the Franka.
    VectorXd actual_q = robot.get_joint_positions();
    std::cout << "actual q: " << actual_q << std::endl;

    // Read input trajectory from file
    //const uint8_t time_offset = 1;
    //std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file("../../examples/data/franka_cartesian_trajectory_circle.csv");
    Pose start_pose = robot.get_target_tcp_pose();
    std::cout << "Robot current pose: " << start_pose.to_string() << std::endl;

    robot.set_control_mode(FrankaRobot::ControlMode::CARTESIAN_POSE);
    robot.set_cartesian_pose_ref(start_pose);
    robot.start_control();
    robot.step();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    double step_time = robot.get_step_time();

    // Control loop
    while (step_time < 10.0)
    {
      auto t_cycle_start = steady_clock::now();
      step_time = robot.get_step_time();

      constexpr double radius = 0.3;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * step_time));
      double delta_x = radius * std::sin(angle);
      double delta_z = radius * (std::cos(angle) - 1);

      /*Pose target_pose = start_pose;
      Vector3d new_position = target_pose.get_position();
      new_position[0] += delta_x;
      new_position[2] += delta_z;
      target_pose.set_position(new_position);

      robot.set_cartesian_pose_ref(target_pose);*/

      franka::CartesianPose start_pose = robot.get_current_pose().O_T_EE;
      Eigen::Affine3d start_pose_transform(Eigen::Matrix4d::Map(start_pose.O_T_EE.data()));
      Pose my_start_pose(start_pose_transform);
      std::cout << "start_pose: " << my_start_pose.to_string() << std::endl;

      std::array<double, 16> new_pose_arr = robot.get_current_pose().O_T_EE;
      new_pose_arr[12] += delta_x;
      new_pose_arr[14] += delta_z;
      franka::CartesianPose new_pose(new_pose_arr);
      robot.set_current_pose(new_pose);
      robot.step();

      auto t_app_stop = steady_clock::now();
      auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
      if (t_app_duration.count() < dt)
      {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_app_duration.count()));
      }
    }
    robot.stop_control();
  }
  catch (const franka::Exception& e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
