add_library(sdu_controllers sdu_controllers.cpp)
target_include_directories(
  sdu_controllers PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/>
                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
