# Define a test executable
add_executable(test_smoke
	test_smoke.cpp
)

# Link the test against the core library so tests can exercise library code
target_link_libraries(test_smoke
	PRIVATE 
	aerosyssim
)

# Register the executable as a CTest test
add_test(NAME smoke COMMAND test_smoke)

# put test binaries alongside other executables
set_target_properties(test_smoke PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)
