add_executable(tests tests.cpp)
target_link_libraries(tests PUBLIC sdu_controllers Catch2::Catch2)
target_compile_definitions(tests PRIVATE PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(rnea_test rnea_test.cpp)
target_link_libraries(rnea_test PUBLIC sdu_controllers Catch2::Catch2)

add_executable(ur5e_inv_dyn_comparison ur5e_inv_dyn_comparison.cpp)
target_link_libraries(ur5e_inv_dyn_comparison PUBLIC sdu_controllers Catch2::Catch2)

add_executable(pose_test pose_test.cpp)
target_link_libraries(pose_test PUBLIC sdu_controllers Catch2::Catch2)
target_compile_definitions(pose_test PRIVATE PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# allow user to run tests with `make test` or `ctest`
catch_discover_tests(tests)
catch_discover_tests(pose_test)
