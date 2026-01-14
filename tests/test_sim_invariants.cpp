#include <cmath>
#include <iostream>
#include <iomanip>

#include "aerosyssim/sim/metrics.hpp"
#include "aerosyssim/sim/sim_engine.hpp"

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_invariants: FAIL: " << msg << "\n";
	return 1;
}

} // namespace

int main() {
	using aerosyssim::math::Mat3;
	using aerosyssim::math::Quat;
	using aerosyssim::math::Vec3;
	using aerosyssim::sim::AttitudeControl;
	using aerosyssim::sim::AttitudeControlFn;
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::SimConfigFixedStep;

	// Torque-free case: tau = 0
	const AttitudeControlFn u_of_t_x = [](double /*t*/, const AttitudeState& /*x*/) {
		return AttitudeControl{Vec3{0.0, 0.0, 0.0}};
	};

	AttitudeState x0;
	x0.q_wxyz = Quat{1.0, 0.0, 0.0, 0.0};
	x0.w_body = Vec3{0.6, 0.4, -0.3};

	// Full (non-diagonal) SPD inertia to exercise the full-inertia RHS path.
	RigidBodyParams p{{2.0, 1.5, 1.2}};
	p.inertia_body = Mat3{
		2.0, 0.1, 0.0,
		0.1, 1.5, 0.0,
		0.0, 0.0, 1.2
	};

	SimConfigFixedStep cfg;
	cfg.t0 = 0.0;
	cfg.dt = 5.0e-4;
	cfg.num_steps = 4000;
	cfg.include_initial = true;

	const auto tr = aerosyssim::sim::run_attitude_fixed_step(cfg, x0, u_of_t_x, p);
	const auto stats = aerosyssim::sim::compute_trace_invariants(tr, p);

	auto dump = [&]() {
		std::cerr.setf(std::ios::scientific);
		std::cerr << std::setprecision(17);
		std::cerr << "sim_invariants diagnostics:\n";
		std::cerr << "qnorm_max_abs_err = " << stats.qnorm_max_abs_err << "\n";
		std::cerr << "energy0           = " << stats.energy0 << "\n";
		std::cerr << "energy_max_abs_err = " << stats.energy_max_abs_err << "\n";
		std::cerr << "energy_rel_change = " << stats.energy_rel_change << "\n";
		std::cerr << "Lnorm0 = " << stats.Lnorm0 << "\n";
		std::cerr << "Lnorm_max_abs_err = " << stats.Lnorm_max_abs_err << "\n";
		std::cerr << "Lnorm_rel_change = " << stats.Lnorm_rel_change << "\n";
	};

	// q is renormalized every step: deviation should be at floating epsilon magnitude
	if (!(stats.qnorm_max_abs_err <= 1.0e-12)) {
		dump();
		return fail("qnorm max abs error too large");
	}

	// For torque-free rigid body, rotational kinetic energy and |L| should be conserved
	if (!(stats.energy_rel_change <= 1.0e-10)) {
		dump();
		return fail("rotational kinetic energy change too large");
	}
	if (!(stats.Lnorm_rel_change <= 1.0e-10)) {
		dump();
		return fail("angular momentum magnitude change too large");
	}

	return 0;
}
