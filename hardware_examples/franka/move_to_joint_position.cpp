#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <sdu_controllers/hal/franka_robot.hpp>
#include <sdu_controllers/math/pose.hpp>

using namespace Eigen;
using namespace sdu_controllers::hal;
using namespace sdu_controllers::math;

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
  }
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
