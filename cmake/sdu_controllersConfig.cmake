include("${CMAKE_CURRENT_LIST_DIR}/sdu_controllers-config.cmake")

set(SDU_CONTROLLERS_LIBRARIES ML::MachineLearning)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(sdu_controllers DEFAULT_MSG SDU_CONTROLLERS_LIBRARIES)

