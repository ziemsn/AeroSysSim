# Define the core library target. Everything reusable goes here.
add_library(aerosyssim STATIC)

# Associate implementation source files with the library target.
target_sources(aerosyssim
	PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/aerosyssim.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/math/quat.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/math/linalg.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dynamics/rigid_body.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/sim/sim_engine.cpp
)

# Export the public header location to anything that links this library.
target_include_directories(aerosyssim
	PUBLIC
	${PROJECT_SOURCE_DIR}/include
)

# Require C++17 for this library and its consumers.
target_compile_features(aerosyssim
	PUBLIC
	cxx_std_17
)
