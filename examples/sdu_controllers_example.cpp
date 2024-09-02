#include <iostream>
#include <Eigen/Dense>
#include <sdu_controllers/sdu_controllers.hpp>

using Eigen::MatrixXd;

int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  const int result = sdu_controllers::add_one(1);
  std::cout << "1 + 1 = " << result << std::endl;
}
