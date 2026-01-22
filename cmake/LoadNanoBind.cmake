# Try to locate nanobind through Python OBS variable must not be named NB_DIR to
# avoid conflict with nanobind's own CMake variables
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE NB_DIR_SDU_CONTROLLERS
  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

# If not found, check in virtual environment
if(NOT NB_DIR_SDU_CONTROLLERS AND DEFINED ENV{VIRTUAL_ENV})
  set(VENV_NANOBIND_DIR
      "$ENV{VIRTUAL_ENV}/lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/nanobind/cmake"
  )
  if(EXISTS "${VENV_NANOBIND_DIR}")
    set(NB_DIR_SDU_CONTROLLERS "${VENV_NANOBIND_DIR}")
    message(STATUS "Found nanobind in VIRTUAL_ENV")
  endif()
endif()

set(NB_FOUND_VIA_PYTHON TRUE)

if(NB_DIR_SDU_CONTROLLERS)
  message(STATUS "Nanobind DIR: ${NB_DIR_SDU_CONTROLLERS}")
  list(APPEND CMAKE_PREFIX_PATH "${NB_DIR_SDU_CONTROLLERS}")

  find_package(nanobind REQUIRED)
else()
  message(
    STATUS "Nanobind python module not found. Installing with git submodule")

  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanobind/CMakeLists.txt)
    message(
      FATAL_ERROR
        "nanobind submodule not found. Please run 'git submodule update --init --recursive'."
    )
  endif()

  add_subdirectory(ext/nanobind)

  set(NB_FOUND_VIA_PYTHON FALSE)

endif()
