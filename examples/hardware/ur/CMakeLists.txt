# example dependencies
find_package(ur_rtde REQUIRED)

add_example(admittance_controller_trajectory cpp ur_rtde::rtde)
add_example(admittance_controller_kinesthetic_teaching cpp ur_rtde::rtde)
add_example(ur_joint_motion_controller cpp ur_rtde::rtde)
