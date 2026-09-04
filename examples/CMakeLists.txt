set(SDU_EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR})

macro(add_example example_name folder)
    add_executable(${example_name} ${folder}/${example_name}.cpp)
    target_link_libraries(${example_name} PRIVATE sdu_controllers ${ARGN})
    target_include_directories(${example_name} PRIVATE ${SDU_EXAMPLES_DIR})
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
