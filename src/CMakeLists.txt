add_library(sdu_controllers
        sdu_controllers.cpp
        ../include/sdu_controllers/controllers/controller.hpp
        controllers/admittance_controller_position.cpp
        ../include/sdu_controllers/controllers/admittance_controller_position.hpp)
target_include_directories(
  sdu_controllers PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/>
                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
