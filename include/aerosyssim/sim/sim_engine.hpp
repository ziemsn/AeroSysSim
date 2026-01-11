#pragma once

#include <cstddef>
#include <functional>
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

// Control callback for time-varying torque policies.
// Evaluate at the start of each integration step with (t, x)
using AttitudeControlFn = std::function<AttitudeControl(double t, const AttitudeState& x)>;

// Runs a deterministic fixed-step attitude simulation in memory.
// Uses step_attitude_rk4 internally (quaternion normalization each step)
SimTraceAttitude run_attitude_fixed_step(
		const SimConfigFixedStep& cfg,
		const AttitudeState& x0,
		const AttitudeControl& u,
		const RigidBodyParams& p);

// Time-varying control overload
SimTraceAttitude run_attitude_fixed_step(
		const SimConfigFixedStep& cfg,
		const AttitudeState& x0,
		const AttitudeControlFn& u_of_t_x,
		const RigidBodyParams& p);


} // namespace aerosyssim::sim
