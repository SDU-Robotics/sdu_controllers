set(sdu_controllers_SOURCES
      sdu_controllers.cpp
      controllers/admittance_controller_position.cpp
      controllers/pd_controller.cpp
      controllers/operational_space_controller.cpp
      controllers/force_control_inner_velocity_loop.cpp
      math/inverse_dynamics_joint_space.cpp
      math/forward_dynamics.cpp
      math/rnea.cpp
      models/breeding_blanket_handling_robot.cpp
      models/breeding_blanket_handling_robot_model.cpp
      models/ur_robot_model.cpp
      models/ur_robot.cpp
      safety/safety_verifier.cpp

)

set(sdu_controllers_HEADERS
      ../include/sdu_controllers/hal/robot.hpp
      ../include/sdu_controllers/controllers/controller.hpp
      ../include/sdu_controllers/controllers/admittance_controller_position.hpp
      ../include/sdu_controllers/kinematics/forward_kinematics.hpp
      ../include/sdu_controllers/math/math.hpp
      ../include/sdu_controllers/math/forward_dynamics.hpp
      ../include/sdu_controllers/math/inverse_dynamics.hpp
      ../include/sdu_controllers/math/inverse_dynamics_joint_space.hpp
      ../include/sdu_controllers/models/breeding_blanket_handling_robot.hpp
      ../include/sdu_controllers/models/breeding_blanket_handling_robot_model.hpp
      ../include/sdu_controllers/models/ur_robot_model.hpp
      ../include/sdu_controllers/controllers/pd_controller.hpp
      ../include/sdu_controllers/models/ur_robot.hpp
      ../include/sdu_controllers/utils/utility.hpp
      ../include/sdu_controllers/safety/safety_verifier.hpp
      ../include/sdu_controllers/controllers/operational_space_controller.hpp
      ../include/sdu_controllers/controllers/force_control_inner_velocity_loop.hpp
      ../include/sdu_controllers/math/rnea.hpp
      ../include/sdu_controllers/math/pose.hpp
)

find_package(ur_rtde QUIET)

if(ur_rtde_FOUND)
    list(APPEND sdu_controllers_SOURCES hal/ur_robot.cpp)
    list(APPEND sdu_controllers_HEADERS ../include/sdu_controllers/hal/ur_robot.hpp)
endif(ur_rtde_FOUND)

add_library(sdu_controllers ${sdu_controllers_SOURCES} ${sdu_controllers_HEADERS})
target_link_libraries(sdu_controllers Eigen3::Eigen)
target_include_directories(
  sdu_controllers PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/>
                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
