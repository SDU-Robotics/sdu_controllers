# example dependencies
find_package(Franka 0.9.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_example(move_to_joint_position cpp Franka::Franka Eigen3::Eigen pinocchio::pinocchio)
add_example(cartesian_control cpp Franka::Franka Eigen3::Eigen pinocchio::pinocchio)

