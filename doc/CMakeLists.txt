find_package(Doxygen REQUIRED)
set(DOXYGEN_EXCLUDE_PATTERNS 
  "${CMAKE_SOURCE_DIR}/ext/*"
  "${CMAKE_SOURCE_DIR}/_skbuild/*"
  "${CMAKE_SOURCE_DIR}/build/*"
  "${CMAKE_SOURCE_DIR}/cmake-build-debug/*"
  "${CMAKE_SOURCE_DIR}/*.egg-info/*"
  "*/eigen/*"
  "*/yaml-cpp/*"
  "*/.git/*"
  "*/.svn/*"
  "*/.hg/*"
  "*/CMakeFiles/*"
  "*/_CPack_Packages/*"
  "*/__pycache__/*"
  "*.pyc"
  "DartConfiguration.tcl"
  "CMakeLists.txt"
  "CMakeCache.txt"
)
set(DOXYGEN_SHORT_NAMES YES)
set(DOXYGEN_GENERATE_XML YES)
doxygen_add_docs(
  doxygen ${CMAKE_SOURCE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT Building doxygen documentation...)
add_custom_target(
  sphinx-doc
  COMMAND
    sphinx-build -b html
    -Dbreathe_projects.sdu_controllers="${CMAKE_CURRENT_BINARY_DIR}/xml" -c
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/sphinx
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating documentation with Sphinx...")
add_dependencies(sphinx-doc doxygen)
