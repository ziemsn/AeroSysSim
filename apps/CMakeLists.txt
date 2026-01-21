# Define an executable (an app). Apps contain main().
add_executable(sim_runner sim_runner.cpp)
add_executable(platform_runner platform_runner.cpp)

# Link the app against the core library. 
target_link_libraries(sim_runner PRIVATE aerosyssim)
target_link_libraries(platform_runner PRIVATE aerosyssim)

# Place the executable in build/bin/
set_target_properties(sim_runner PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(platform_runner PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)
