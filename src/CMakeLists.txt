set(sdu_controllers_SOURCES
      sdu_controllers.cpp
      controllers/admittance_controller_position.cpp
      controllers/pid_controller.cpp
      controllers/operational_space_controller.cpp
      controllers/force_control_inner_velocity_loop.cpp
      kinematics/forward_kinematics.cpp
      kinematics/dh_kinematics.cpp
      math/inverse_dynamics_joint_space.cpp
      math/forward_dynamics.cpp
      math/rnea.cpp
      models/breeding_blanket_handling_robot_model.cpp
      models/ur_robot_model.cpp
      models/regressor_robot_model.cpp
      models/parameter_robot_model.cpp
      safety/safety_verifier.cpp
      utils/utility.cpp
)

# Try to find ur_rtde in order to build UR Robot HAL
find_package(ur_rtde 1.6.2 QUIET)

if(ur_rtde_FOUND)
    list(APPEND sdu_controllers_SOURCES hal/ur_robot.cpp)
endif(ur_rtde_FOUND)

# Try to find libfranka in order to build Franka Robot HAL
find_package(Franka 0.9.1 QUIET)

if(Franka_FOUND)
    list(APPEND sdu_controllers_SOURCES hal/franka_robot.cpp)
endif(Franka_FOUND)

if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

add_library(sdu_controllers ${sdu_controllers_SOURCES})
target_link_libraries(sdu_controllers PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)
target_compile_definitions(sdu_controllers PUBLIC PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_include_directories(
  sdu_controllers PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/>
                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
