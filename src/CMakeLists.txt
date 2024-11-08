add_library(sdu_controllers
        sdu_controllers.cpp
        controllers/admittance_controller_position.cpp
        controllers/pd_controller.cpp
        math/inverse_dynamics_joint_space.cpp
        math/forward_dynamics.cpp
        models/breeding_blanket_handling_robot_model.cpp
        models/ur_robot_model.cpp
        models/ur_robot.cpp
        safety/safety_verifier.cpp
        controllers/operational_space_controller.cpp
        ../include/sdu_controllers/controllers/controller.hpp
        ../include/sdu_controllers/controllers/admittance_controller_position.hpp
        ../include/sdu_controllers/kinematics/forward_kinematics.hpp
        ../include/sdu_controllers/math/math.hpp
        ../include/sdu_controllers/math/forward_dynamics.hpp
        ../include/sdu_controllers/math/inverse_dynamics.hpp
        ../include/sdu_controllers/math/inverse_dynamics_joint_space.hpp
        ../include/sdu_controllers/models/breeding_blanket_handling_robot_model.hpp
        ../include/sdu_controllers/models/ur_robot_model.hpp
        ../include/sdu_controllers/controllers/pd_controller.hpp
        ../include/sdu_controllers/models/ur_robot.hpp
        ../include/sdu_controllers/utils/utility.hpp
        ../include/sdu_controllers/safety/safety_verifier.hpp
        ../include/sdu_controllers/controllers/operational_space_controller.hpp

)
target_link_libraries(sdu_controllers Eigen3::Eigen)
target_include_directories(
  sdu_controllers PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/>
                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
