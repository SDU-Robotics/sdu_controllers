add_executable(tests tests.cpp sdu_controllers_t.cpp)
target_link_libraries(tests PUBLIC sdu_controllers Catch2::Catch2)

# allow user to run tests with `make test` or `ctest`
catch_discover_tests(tests)
