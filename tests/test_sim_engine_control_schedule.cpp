#include <cmath>
#include <iostream>

#include "aerosyssim/sim/control.hpp"
#include "aerosyssim/sim/sim_engine.hpp"

namespace {
	
	int fail(const char* msg) {
		std::cerr << "test_sim_engine_control_schedule: FAIL: " << msg << "\n";
		return 1;
	}

	bool near(double a, double b, double tol) {
		return std::fabs(a - b) <= tol;
	}

} // namespace

int main() {
	using aerosyssim::math::Vec3;
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::PiecewiseConstantTorqueSchedule;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::SimConfigFixedStep;
	using aerosyssim::sim::run_attitude_fixed_step;

	SimConfigFixedStep cfg;
	cfg.t0 = 0.0;
	cfg.dt = 0.01;
	cfg.num_steps = 100;
	cfg.include_initial = true;

	AttitudeState x0;
	x0.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
	x0.w_body = Vec3{0.0, 0.0, 0.0};

	// Identity inertia = >wdot = tau (since w x (I w) = w x w = 0)
	const RigidBodyParams p{Vec3{1.0, 1.0, 1.0}};

	PiecewiseConstantTorqueSchedule sched;
	sched.t_breaks = {0.5};
	sched.torque_body = {Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 2.0, 0.0}};
	if (!sched.is_valid()) {
		return fail("schedule invariant violated");
	}

	const auto trace = run_attitude_fixed_step(cfg, x0, sched, p);

	if (trace.x.size() != (cfg.num_steps + 1)) {
		return fail("unexpected trace length");
	}

	const auto& xf = trace.x.back();
	const double wx = xf[4];
	const double wy = xf[5];
	const double wz = xf[6];

	// 0.5 seconds of tau=[1,0,0] => wx=0.5
	// 0.5 seconds of tau=[0,2,0] => wy=1.0
	const double tol = 1e-12;
	if (!near(wx, 0.5, tol)) {
		return fail("wx did no match expected piecewise result");
	}
	if (!near(wy, 1.0, tol)) {
		return fail("wy did no match expected piecewise result");
	}
	if (!near(wz, 0.0, tol)) {
		return fail("wz did no match expected piecewise result");
	}
	
	return 0;
}
