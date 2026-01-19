#pragma once

#include <cstddef>
#include <vector>

#include "aerosyssim/sim/platform_stepper_1d.hpp"
#include "aerosyssim/sim/platform_types.hpp"

namespace aerosyssim::sim {

struct SimTracePlatform1D {
	std::vector<double> t;
	std::vector<PackedPlatform1DState> x;
	std::vector<double> a_cmd;
};

inline PackedPlatform1DState pack_platform1d_state(const Platform1DState& s) {
	return PackedPlatform1DState{s.d, s.vd};
}

inline SimTracePlatform1D run_platform1d_fixed_step_constant_control(
	const double t0,
	const double dt,
	const int num_steps,
	const bool include_initial,
	const Platform1DState& x0,
	const Platform1DControl& u,
	const Platform1DParams& p
) {
	SimTracePlatform1D tr{};
	const int n_samples = include_initial ? (num_steps + 1) : num_steps;
	tr.t.reserve(static_cast<size_t>(n_samples));
	tr.x.reserve(static_cast<size_t>(n_samples));
	tr.a_cmd.reserve(static_cast<size_t>(n_samples));

	double t = t0;
	Platform1DState x = x0;

	if(include_initial) {
		tr.t.push_back(t);
		tr.x.push_back(pack_platform1d_state(x));
		tr.a_cmd.push_back(u.a_cmd);
	}

	for (int k = 0; k < num_steps; ++k) {
		x = step_platform1d_rk4(dt, x, u, p);
		t += dt;
		tr.t.push_back(t);
		tr.x.push_back(pack_platform1d_state(x));
		tr.a_cmd.push_back(u.a_cmd);
	}

	return tr;
}

} // namespace aerossyssim::sim
