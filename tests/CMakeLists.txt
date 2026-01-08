# Define a test executable
add_executable(test_smoke
	test_smoke.cpp
)

add_executable (test_quat 
	test_quat.cpp
)

add_executable (test_math_skew
	test_math_skew.cpp
)

add_executable (test_integrator_rk4
	test_integrator_rk4.cpp
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

target_link_libraries(test_math_skew
	PRIVATE
	aerosyssim
)

target_link_libraries(test_integrator_rk4
	PRIVATE
	aerosyssim
)

# Register the executable as a CTest test
add_test(NAME smoke COMMAND test_smoke)
add_test(NAME quat COMMAND test_quat)
add_test(NAME math_skew COMMAND test_math_skew)
add_test(NAME integrator_rk4 COMMAND test_integrator_rk4)

# put test binaries alongside other executables
set_target_properties(test_smoke PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_quat PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_math_skew PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_integrator_rk4 PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)
