# Try to locate nanobind through Python
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE NB_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

# If not found, check in virtual environment
if(NOT NB_DIR AND DEFINED ENV{VIRTUAL_ENV})
  set(VENV_NANOBIND_DIR
      "$ENV{VIRTUAL_ENV}/lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/nanobind/cmake"
  )
  if(EXISTS "${VENV_NANOBIND_DIR}")
    set(NB_DIR "${VENV_NANOBIND_DIR}")
    message(STATUS "Found nanobind in VIRTUAL_ENV")
  endif()
endif()

set(NB_FOUND_VIA_PYTHON TRUE)

if(NB_DIR)
  message(STATUS "Nanobind DIR: ${NB_DIR}")
  list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")

  find_package(nanobind REQUIRED)
else()
  message(
    STATUS "Nanobind python module not found. Installing with git submodule")

  set(NB_FOUND_VIA_PYTHON FALSE)
  # Add Python bindings
  add_subdirectory(${CMAKE_SOURCE_DIR}/ext/nanobind)
endif()
