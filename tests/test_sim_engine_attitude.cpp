#include "aerosyssim/sim/sim_engine.hpp"

#include <cstring>
#include <iostream>

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_engine_attitude: FAIL: " << msg << "\n";
	return 1;
}

} // namespace
  
int main() {
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::AttitudeControl;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::SimConfigFixedStep;

	AttitudeState x0;
	x0.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
	x0.w_body = aerosyssim::math::Vec3{0.3, -0.2, 0.1};

	const AttitudeControl u{{0.0, 0.0, 0.0}};
	const RigidBodyParams p{{2.0, 3.0, 4.0}};

	SimConfigFixedStep cfg;

	cfg.t0 = 0.0;
	cfg.dt = 0.01;
	cfg.num_steps = 50;
	cfg.include_initial = true;

	const auto a = aerosyssim::sim::run_attitude_fixed_step(cfg, x0, u, p);
	const auto b = aerosyssim::sim::run_attitude_fixed_step(cfg, x0, u, p);

	if (a.t.size() != b.t.size() || a.x.size() != b.x.size()) {
		return fail("trace sizes differe for identical inputs");
	}

	if (a.t.empty() || b.t.empty()) {
		return fail("trace should not be empty");
	}

	// Byte-level determinism on packed states
	for (std::size_t i = 0; i < a.x.size(); ++i) {
		if (std::memcmp(a.x[i].data(), b.x[i].data(), sizeof(double) * a.x[i].size()) != 0) {
			return fail("packed state differs between identical runs");	
		}
	}

	// Exact time determinism for this arithmetic pattern
	for (std::size_t i = 0; i < a.t.size(); ++i) {
		if (a.t[i] != b.t[i]) {
			return fail("time vector differs between identical runs");
		}
	}

	return 0;
}
