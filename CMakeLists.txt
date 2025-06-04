cmake_minimum_required(VERSION 3.28)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a name and a version number for your project:
project(
  sdu_controllers
  VERSION 0.0.2
  LANGUAGES CXX)

# Initialize some default paths
include(GNUInstallDirs)

# Define the minimum C++ standard that is required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Compilation options
option(BUILD_PYTHON "Enable building of Python bindings" ON)
option(BUILD_DOCS "Enable building of documentation" ON)
option(BUILD_EXAMPLES "Enable building of examples" ON)
option(BUILD_TESTING "Enable building of the tests" ON)

# Optional hardware examples
option(BUILD_UR_EXAMPLES "Enable building of examples to be run on UR robots" OFF)
option(BUILD_FRANKA_EXAMPLES "Enable building of examples to be run on Franka robots" OFF)

# Add config library as a path such that we can find parameter files used for the robot models.
file(REAL_PATH "./config" CONFIG_FILE_PATH)
file(TO_CMAKE_PATH "${CONFIG_FILE_PATH}" CONFIG_FILE_PATH_NORMALIZED)
add_definitions(-DCONFIG_FILE_PATH="${CONFIG_FILE_PATH_NORMALIZED}")

include(CMakePrintHelpers)
cmake_print_variables(CONFIG_FILE_PATH_NORMALIZED)

# find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# compile the library
add_subdirectory(src)

# compile the examples
if(BUILD_EXAMPLES)
add_subdirectory(examples)
endif()
add_subdirectory(hardware_examples)

# compile the tests
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(ext/Catch2)
  include(./ext/Catch2/contrib/Catch.cmake)
  add_subdirectory(tests)
endif()

if(BUILD_DOCS)
  # Add the documentation
  add_subdirectory(doc)
endif()
if(BUILD_PYTHON)
  if (CMAKE_VERSION VERSION_LESS 3.18)
    set(DEV_MODULE Development)
  else()
    set(DEV_MODULE Development.Module)
  endif()

  find_package(Python 3.8 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)

  # Add Python bindings
  add_subdirectory(ext/nanobind)

  # Compile the nanobind module
  nanobind_add_module(_sdu_controllers
                      python/sdu_controllers/_sdu_controllers.cpp)
  target_link_libraries(_sdu_controllers PUBLIC sdu_controllers)

  # Install the Python module shared library
  install(TARGETS _sdu_controllers DESTINATION .)
endif()

# Add an alias target for use if this project is included as a subproject in
# another project
add_library(sdu_controllers::sdu_controllers ALIAS sdu_controllers)

# Install targets and configuration
install(
  TARGETS sdu_controllers
  EXPORT sdu_controllers-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
  EXPORT sdu_controllers-config
  NAMESPACE sdu_controllers::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sdu_controllers)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This prints a summary of found dependencies
include(FeatureSummary)
feature_summary(WHAT ALL)
