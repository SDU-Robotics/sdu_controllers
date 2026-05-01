cmake_minimum_required(VERSION 3.16)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(POLICY CMP0167) # CMake 3.30 and later
  cmake_policy(SET CMP0167 NEW)
endif()

# Set a name and a version number for your project:
project(
  sdu_controllers
  VERSION 0.0.1
  LANGUAGES CXX)

# Initialize some default paths
include(GNUInstallDirs)

# Define the minimum C++ standard that is required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Compilation options
option(BUILD_PYTHON "Enable building of Python bindings" ON)
option(BUILD_PYTHON_STUBS "Enable building of python Stubs" ${BUILD_PYTHON})
option(BUILD_DOCS "Enable building of documentation" ON)
option(BUILD_HAL "Enable building of Hardware Abstraction Layer" OFF)
option(BUILD_EXAMPLES "Enable building of examples" ON)
option(BUILD_TESTING "Enable building of the tests" ON)
option(BUILD_FOR_PIP_INSTALL "Enable Install commands suitable for pip installations" OFF)

# Optional hardware examples
option(BUILD_UR_EXAMPLES "Enable building of examples to be run on UR robots"
       OFF)
option(BUILD_FRANKA_EXAMPLES
       "Enable building of examples to be run on Franka robots" OFF)

# Add config library as a path such that we can find parameter files used for
# the robot models.

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.20")
  file(REAL_PATH "config" CONFIG_FILE_PATH)
else()
  set(CONFIG_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

file(TO_CMAKE_PATH "${CONFIG_FILE_PATH}" CONFIG_FILE_PATH_NORMALIZED)
add_definitions(-DCONFIG_FILE_PATH="${CONFIG_FILE_PATH_NORMALIZED}")

include(CMakePrintHelpers)
#cmake_print_variables(CONFIG_FILE_PATH_NORMALIZED)

# Find dependencies
include(FetchContent)
# --- Fetch Eigen3 ---
set(EIGEN_BUILD_TESTING OFF CACHE BOOL " " FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL " " FORCE)
set(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF CACHE BOOL " " FORCE)
set(EIGEN_GIT_TAG 3.4.0 CACHE STRING " " FORCE)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        ${EIGEN_GIT_TAG}
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()

# Manually create the header-only target exactly as Eigen would
add_library(Eigen3::Eigen INTERFACE IMPORTED)
target_include_directories(Eigen3::Eigen INTERFACE "${eigen_SOURCE_DIR}")
message(STATUS "Found Eigen3: ${eigen_SOURCE_DIR} (found version \"${EIGEN_GIT_TAG}\")")

# --- Fetch yaml-cpp ---
SET(YAML_CPP_INSTALL ON CACHE INTERNAL "")
# Force static linking so the wheel is self-contained
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL " " FORCE)
# Disable yaml-cpp tests and tools to speed up build
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL " " FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL " " FORCE)
set(YAML_CPP_GIT_TAG yaml-cpp-0.9.0 CACHE STRING " " FORCE)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        ${YAML_CPP_GIT_TAG}
)
FetchContent_MakeAvailable(yaml-cpp)
message(STATUS "Found yaml-cpp: ${yaml-cpp_SOURCE_DIR} (found version \"${YAML_CPP_GIT_TAG}\")")

# compile the library
add_subdirectory(src)

# compile the examples
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# compile the tests
include(CTest)
if(BUILD_TESTING)
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ext/Catch2/CMakeLists.txt)
    message(
      FATAL_ERROR
        "Catch2 submodule not found. Please run 'git submodule update --init --recursive'."
    )
  endif()

  add_subdirectory(ext/Catch2)
  include(./ext/Catch2/contrib/Catch.cmake)
  add_subdirectory(tests)
endif()

if(BUILD_DOCS)
  # Add the documentation
  add_subdirectory(doc)
endif()
if(BUILD_PYTHON)

  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanobind/CMakeLists.txt)
    message(
      FATAL_ERROR
        "nanobind submodule not found. Please run 'git submodule update --init --recursive'."
    )
  endif()
  if(CMAKE_VERSION VERSION_LESS 3.18)
    set(DEV_MODULE Development)
  else()
    set(DEV_MODULE Development.Module)
  endif()

  find_package(
    Python 3.8
    COMPONENTS Interpreter ${DEV_MODULE}
    REQUIRED)

  # make sure python executable is set (for older CMake versions that does not set Python3_EXECUTABLE automatically)
  if(NOT EXISTS ${Python3_EXECUTABLE})
    if(EXISTS ${Python_EXECUTABLE})
      set(Python3_EXECUTABLE ${Python_EXECUTABLE})
    else()
      message(FATAL_ERROR "Python 3.8 or higher is required to build the Python bindings.")
    endif()
  endif()


  include(cmake/LoadNanoBind.cmake)
  
  add_subdirectory(python)
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

# Install targets and configuration
install(
  TARGETS yaml-cpp
  EXPORT sdu_controllers-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Create targets file for none install
export(
  EXPORT sdu_controllers-config
  FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdu_controllers-config.cmake"
  NAMESPACE sdu_controllers::)

# install target config file
install(
  EXPORT sdu_controllers-config
  NAMESPACE sdu_controllers::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sdu_controllers)

# Install Config file
install(FILES cmake/sdu_controllersConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sdu_controllers)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Auto find project
if(NOT WIN32)
  file(MAKE_DIRECTORY $ENV{HOME}/.cmake/packages/sdu_controllers)
  file(WRITE "$ENV{HOME}/.cmake/packages/sdu_controllers/sdu_controllers123"
       "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
endif()

# This prints a summary of found dependencies
include(FeatureSummary)
feature_summary(WHAT ALL)
