# Compile the nanobind module
nanobind_add_module(
  _sdu_controllers sdu_controllers/_sdu_controllers.cpp
  sdu_controllers/_robot_models.cpp sdu_controllers/_math.cpp
  sdu_controllers/_kinematics.cpp)
target_link_libraries(_sdu_controllers PUBLIC sdu_controllers)

# install python bindings if building for pip install
if(BUILD_FOR_PIP_INSTALL)
  # Install the Python module shared library
  install(TARGETS _sdu_controllers DESTINATION .)
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/config DESTINATION .)

# Check if nanobind is available as a Python import
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import nanobind"
  RESULT_VARIABLE nanobind_NOT_FOUND
  OUTPUT_QUIET ERROR_QUIET)

if(BUILD_PYTHON_STUBS AND NOT nanobind_NOT_FOUND)

  # Automatic stub generation
  add_custom_command(
    TARGET _sdu_controllers
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Generating Python type stubs... 1"
    COMMAND
      ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/generate_stubs.py
      ${CMAKE_CURRENT_BINARY_DIR}/generate_stubs.py
    COMMAND ${Python3_EXECUTABLE} ./generate_stubs.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating sdu_controllers type stubs for IDE support"
    VERBATIM)

  if(BUILD_FOR_PIP_INSTALL)
    # Install all generated .pyi stub files (at install time, after generation)
    install(
      CODE "file(GLOB STUB_FILES \"${CMAKE_CURRENT_BINARY_DIR}/*.pyi\")
            foreach(stub_file \${STUB_FILES})
              get_filename_component(stub_basename \${stub_file} NAME_WE)
              if(stub_basename STREQUAL \"sdu_controllers\")
                file(INSTALL \${stub_file} DESTINATION \${CMAKE_INSTALL_PREFIX}/ RENAME __init__.pyi)
              else()
                file(INSTALL \${stub_file} DESTINATION \${CMAKE_INSTALL_PREFIX}/\${stub_basename} RENAME __init__.pyi)
              endif()
            endforeach()")

    # Generate py.typed marker for PEP 561 compliance
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/py.typed "")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/py.typed DESTINATION .)
  endif()
elseif(nanobind_NOT_FOUND AND BUILD_PYTHON_STUBS)
  message(
    WARNING
      "nanobind Python package not found; skipping generation of type stubs")

endif()
