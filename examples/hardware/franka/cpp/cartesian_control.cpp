#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
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
    std::cout << "actual q: " << actual_q;

    // Read input trajectory from file
    const uint8_t time_offset = 1;
    std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file("../../examples/data/franka_cartesian_trajectory_circle.csv");

    // First generate trajectory that moves the robot to the start of the circle to avoid any jumps.
    //std::vector<double> start_pose_vec(7);
    //start_pose_vec = {3.06890567e-01, -7.42545463e-17, 4.86882052e-01, 4.32978028e-17, 1.00000000e+00, -1.11022302e-16, -3.06161700e-17};
    //Pose start_pose(start_pose_vec);

    /*Pose start_pose = robot.get_cartesian_tcp_pose();
    std::vector<double> x_d_init_vec(6);
    for (Index i = 0; i < x_d_init_vec.size(); i++)
    {
      x_d_init_vec[i] = input_trajectory[0][time_offset + i];
    }
    Pose target_pose(x_d_init_vec);
    double total_time = 3.0;
    double acceleration_time = 0.5;
    std::vector<Pose> init_trajectory = generate_trapezoidal_trajectory(start_pose, target_pose, total_time, acceleration_time);*/

    robot.set_control_mode(FrankaRobot::ControlMode::CARTESIAN_POSE);
    robot.start_control();

    /*for (const auto& pose : init_trajectory)
    {
      auto t_cycle_start = steady_clock::now();
      robot.set_cartesian_pose_ref(pose);
      robot.step();

      auto t_app_stop = steady_clock::now();
      auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
      if (t_app_duration.count() < dt)
      {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_app_duration.count()));
      }
      }*/

    // Control loop
    for (const std::vector<double> &pose : input_trajectory)
    {
      auto t_cycle_start = steady_clock::now();

      std::vector<double> x_d_vec(6);

      // Get desired cartesian pose
      for (Index i = 0; i < x_d_vec.size(); i++)
      {
        x_d_vec[i] = pose[time_offset + i];
      }
      Pose x_d(x_d_vec);
      std::cout << "x_d: " << x_d.to_string() << std::endl;
      robot.set_cartesian_pose_ref(x_d);
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
