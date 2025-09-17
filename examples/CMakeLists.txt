if(BUILD_EXAMPLES)
    add_subdirectory(simulation)
endif()

if(BUILD_UR_EXAMPLES)
    add_subdirectory(hardware/ur)
endif()

if(BUILD_FRANKA_EXAMPLES)
    add_subdirectory(hardware/franka)
endif()
