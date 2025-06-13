# example dependencies
find_package(Franka 0.9.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(move_to_joint_position move_to_joint_position.cpp)
target_link_libraries(move_to_joint_position PRIVATE sdu_controllers Franka::Franka Eigen3::Eigen pinocchio::pinocchio)

add_executable(cartesian_control cartesian_control.cpp)
target_link_libraries(cartesian_control PRIVATE sdu_controllers Franka::Franka Eigen3::Eigen pinocchio::pinocchio)
