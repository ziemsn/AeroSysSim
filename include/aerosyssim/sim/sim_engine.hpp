#pragma once

#include <cstddef>
#include <vector>

#include "aerosyssim/sim/types.hpp"

namespace aerosyssim::sim {

struct SimTraceAttitude {
	std::vector<double> t;
	std::vector<PackedAttitudeState> x;
};

struct SimConfigFixedStep {
	double t0{0.0};
	double dt{0.01};
	std::size_t num_steps{0}; // number of integration steps
	bool include_initial{true}; // if true, trace includes state at t0
};

// Runs a deterministic fixed-step attitude simulation in memory.
// Uses step_attitudue_rk4 internally (quaternion renormalized each step).
SimTraceAttitude run_attitude_fixed_step(
		const SimConfigFixedStep& cfg,
		const AttitudeState& x0,
		const AttitudeControl& u,
		const RigidBodyParams& p);

} // namespace aerosyssim::sim
