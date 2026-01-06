# Define a test executable
add_executable(test_smoke
	test_smoke.cpp
)

add_executable (test_quat 
	test_quat.cpp
)

# Link the test against the core library so tests can exercise library code
target_link_libraries(test_smoke
	PRIVATE 
	aerosyssim
)

target_link_libraries(test_quat
	PRIVATE
	aerosyssim
)

# Register the executable as a CTest test
add_test(NAME smoke COMMAND test_smoke)

add_test(NAME quat COMMAND test_quat)

# put test binaries alongside other executables
set_target_properties(test_smoke PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)
