macro(add_example example_name folder)
    add_executable(${example_name} ${folder}/${example_name}.cpp)
    target_link_libraries(${example_name} PRIVATE sdu_controllers ${ARGN})
endmacro()


if(BUILD_EXAMPLES)
    add_subdirectory(simulation)
endif()

if(BUILD_UR_EXAMPLES)
    add_subdirectory(hardware/ur)
endif()

if(BUILD_FRANKA_EXAMPLES)
    add_subdirectory(hardware/franka)
endif()
