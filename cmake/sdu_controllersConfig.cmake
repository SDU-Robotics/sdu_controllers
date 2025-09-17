include("${CMAKE_CURRENT_LIST_DIR}/sdu_controllers-config.cmake")
set(SDU_CONTROLLERS_LIBRARIES sdu_controllers::sdu_controllers)

# Required dependencies
find_package(yaml-cpp CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(sdu_controllers DEFAULT_MSG SDU_CONTROLLERS_LIBRARIES)

