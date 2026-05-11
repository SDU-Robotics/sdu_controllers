# CMake script to copy the module during installation
# This is executed during the install step on Windows
# Variables available from CMake during install: CMAKE_INSTALL_PREFIX, CMAKE_INSTALL_CONFIG_NAME

set(COPY_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/copy_module.py")
set(INSTALL_DIR "${CMAKE_INSTALL_PREFIX}")

if(NOT EXISTS "${COPY_SCRIPT}")
  message(WARNING "copy_module.py not found at ${COPY_SCRIPT}")
else()
  execute_process(
    COMMAND ${Python3_EXECUTABLE} "${COPY_SCRIPT}" "${INSTALL_DIR}"
    RESULT_VARIABLE copy_result
  )
  
  if(NOT copy_result EQUAL 0)
    message(WARNING "Failed to copy module to standard name during installation")
  endif()
endif()
