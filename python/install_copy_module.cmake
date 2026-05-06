# CMake script to copy the module during installation
# This is executed during the install step on Windows

execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/copy_module.py ${CMAKE_INSTALL_PREFIX}
  RESULT_VARIABLE copy_result
)

if(NOT copy_result EQUAL 0)
  message(WARNING "Failed to copy module to standard name during installation")
endif()
