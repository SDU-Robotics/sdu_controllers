add_library(sdu_controllers
        sdu_controllers.cpp
        controllers/admittance_controller_position.cpp
        math/inverse_dynamics_joint_space.cpp
        ../include/sdu_controllers/controllers/controller.hpp
        ../include/sdu_controllers/controllers/admittance_controller_position.hpp
        ../include/sdu_controllers/math/inverse_dynamics.hpp
        ../include/sdu_controllers/math/inverse_dynamics_joint_space.hpp
        ../include/sdu_controllers/models/breeding_blanket_handling_robot_model.hpp
        models/breeding_blanket_handling_robot_model.cpp)
target_link_libraries(sdu_controllers Eigen3::Eigen)
target_include_directories(
  sdu_controllers PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/>
                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

