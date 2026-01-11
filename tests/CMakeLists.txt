# Define test executables
add_executable(test_smoke test_smoke.cpp)
add_executable (test_quat test_quat.cpp)
add_executable (test_math_skew test_math_skew.cpp)
add_executable (test_integrator_rk4 test_integrator_rk4.cpp)
add_executable (test_types test_types.cpp)
add_executable (test_rigid_body_rhs test_rigid_body_rhs.cpp)
add_executable (test_attitude_stepper test_attitude_stepper.cpp)
add_executable (test_sim_engine_attitude test_sim_engine_attitude.cpp)
add_executable (test_sim_runner_csv test_sim_runner_csv.cpp)
add_executable (test_sim_engine_control_schedule test_sim_engine_control_schedule.cpp)

# Link the tests against the core library so tests can exercise library code
target_link_libraries(test_smoke PRIVATE aerosyssim)
target_link_libraries(test_quat	PRIVATE aerosyssim)
target_link_libraries(test_math_skew PRIVATE aerosyssim)
target_link_libraries(test_integrator_rk4 PRIVATE aerosyssim)
target_link_libraries(test_types PRIVATE aerosyssim)
target_link_libraries(test_rigid_body_rhs PRIVATE aerosyssim)
target_link_libraries(test_attitude_stepper PRIVATE aerosyssim)
target_link_libraries(test_sim_engine_attitude PRIVATE aerosyssim)
target_link_libraries(test_sim_engine_control_schedule PRIVATE aerosyssim)

# Register the executable as a CTest test
add_test(NAME smoke COMMAND test_smoke)
add_test(NAME quat COMMAND test_quat)
add_test(NAME math_skew COMMAND test_math_skew)
add_test(NAME integrator_rk4 COMMAND test_integrator_rk4)
add_test(NAME types COMMAND test_types)
add_test(NAME rigid_body_rhs COMMAND test_rigid_body_rhs)
add_test(NAME attitude_stepper COMMAND test_attitude_stepper)
add_test(NAME sim_engine_attitude COMMAND test_sim_engine_attitude)
add_test(NAME sim_runner_csv COMMAND test_sim_runner_csv)
add_test(NAME sim_engine_control_schedule COMMAND test_sim_engine_control_schedule)

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

set_target_properties(test_types PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_rigid_body_rhs PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_attitude_stepper PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_sim_engine_attitude PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_sim_runner_csv PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

set_target_properties(test_sim_engine_control_schedule PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)
