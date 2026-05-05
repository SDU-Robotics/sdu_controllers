# Try to locate nanobind through Python
# OBS: Variable must not be named NB_DIR to avoid conflict with nanobind's own CMake variables

# First, check if NANOBIND_DIR environment variable is set (useful for CI/cibuildwheel)
if(DEFINED ENV{NANOBIND_DIR})
  set(NB_DIR_SDU_CONTROLLERS "$ENV{NANOBIND_DIR}")
  message(STATUS "Found nanobind from NANOBIND_DIR: ${NB_DIR_SDU_CONTROLLERS}")
endif()

# Try to locate nanobind through Python executable
if(NOT NB_DIR_SDU_CONTROLLERS AND DEFINED Python_EXECUTABLE)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_VARIABLE NB_DIR_SDU_CONTROLLERS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_VARIABLE NB_ERROR
    RESULT_VARIABLE NB_RESULT)
  
  if(NB_RESULT EQUAL 0)
    message(STATUS "Found nanobind via Python: ${NB_DIR_SDU_CONTROLLERS}")
  else()
    message(STATUS "Could not find nanobind via Python: ${NB_ERROR}")
    unset(NB_DIR_SDU_CONTROLLERS)
  endif()
endif()

# If not found, try to get nanobind location from pip
if(NOT NB_DIR_SDU_CONTROLLERS AND DEFINED Python_EXECUTABLE)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import nanobind; import os; print(os.path.dirname(nanobind.__file__))"
    OUTPUT_VARIABLE NB_PYTHON_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE NB_IMPORT_RESULT)
  
  if(NB_IMPORT_RESULT EQUAL 0 AND NB_PYTHON_DIR)
    set(NB_DIR_SDU_CONTROLLERS "${NB_PYTHON_DIR}/cmake")
    if(EXISTS "${NB_DIR_SDU_CONTROLLERS}")
      message(STATUS "Found nanobind cmake directory: ${NB_DIR_SDU_CONTROLLERS}")
    else()
      unset(NB_DIR_SDU_CONTROLLERS)
    endif()
  endif()
endif()

# If not found, check in virtual environment
if(NOT NB_DIR_SDU_CONTROLLERS AND DEFINED ENV{VIRTUAL_ENV})
  set(VENV_NANOBIND_DIR
      "$ENV{VIRTUAL_ENV}/lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/nanobind/cmake"
  )
  if(EXISTS "${VENV_NANOBIND_DIR}")
    set(NB_DIR_SDU_CONTROLLERS "${VENV_NANOBIND_DIR}")
    message(STATUS "Found nanobind in VIRTUAL_ENV: ${NB_DIR_SDU_CONTROLLERS}")
  endif()
endif()

# Additional fallback: try common site-packages locations
if(NOT NB_DIR_SDU_CONTROLLERS)
  # Try using Python to get site-packages
  if(DEFINED Python_EXECUTABLE)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -c "import site; print(site.getsitepackages()[0])"
      OUTPUT_VARIABLE SITE_PACKAGES
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    
    if(SITE_PACKAGES)
      set(POTENTIAL_NB_DIR "${SITE_PACKAGES}/nanobind/cmake")
      if(EXISTS "${POTENTIAL_NB_DIR}")
        set(NB_DIR_SDU_CONTROLLERS "${POTENTIAL_NB_DIR}")
        message(STATUS "Found nanobind in site-packages: ${NB_DIR_SDU_CONTROLLERS}")
      endif()
    endif()
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
