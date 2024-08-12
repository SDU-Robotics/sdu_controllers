#include <iostream>

#include "sdu_controllers/sdu_controllers.hpp"

int main()
{
  int result = sdu_controllers::add_one(1);
  std::cout << "1 + 1 = " << result << std::endl;
}
