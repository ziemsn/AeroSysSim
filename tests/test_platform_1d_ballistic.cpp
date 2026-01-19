#include "aerosyssim/sim/platform_engine_1d.hpp"

#include <cmath>
#include <iostream>

namespace {

int fail(const char* msg) {
	std::cerr << "test_platform_1d_ballistic: FAIL: " << msg << "\n";
	return 1;
}

bool near(const double a, const double b, const double atol=1e-12, const double rtol = 1e-12) {
	return std::fabs(a - b) <= (atol + rtol * std::fabs(b));
}	

} // namespace

int main() {

	// Ballistic case (no thrust): d(t) = d0 + vd0 t + + 0.5 g t^2, vd(t) = vd0 + g t
	{
		const double t0 = 0.0;
		const double dt = 0.01;
		const int steps = 200;
		const bool include_initial = true;

		const auto x0 = aerosyssim::sim::make_platform1d_state(0.0, 1.25);
		const auto u = aerosyssim::sim::make_platform1d_control(0.0);
		auto p = aerosyssim::sim::make_platform1d_params_default();
		p.g = 9.80665;

		const auto tr = aerosyssim::sim::run_platform1d_fixed_step_constant_control(
			t0, dt, steps, include_initial, x0, u, p
		);

		if (static_cast<int>(tr.t.size()) != steps + 1) return fail("trace length should be steps+1 with include_initial");
		if (static_cast<int>(tr.x.size()) != steps + 1) return fail("packed trace length should be steps+1 with include_initial");
		if (static_cast<int>(tr.a_cmd.size()) != steps + 1) return fail("control trace length should be steps+1 with include_initial");

		if (!near(tr.t.front(), t0)) return fail("first time sample shoulud equal t0");
		if (!near(tr.x.front()[0], x0.d) || !near(tr.x.front()[1], x0.vd)) return fail("first state sample should equal x0");

		const double tf = dt * static_cast<double>(steps);
		const double d_expected = x0.d + x0.vd * tf + 0.5 * p.g * tf * tf;
		const double vd_expected = x0.vd + p.g * tf;

		const double d_final = tr.x.back()[0];
		const double vd_final = tr.x.back()[1];

		if(!near(d_final, d_expected, 1e-10, 1e-10)) return fail("final d does not match ballistic analytic solution");
		if(!near(vd_final, vd_expected, 1e-12, 1e-12)) return fail("final vd does not match ballistic analytic solution");
	}

	// Hover case (a_cmd = g): vd constant, d linear.
	{
		const double t0 = 0.0;
		const double dt = 0.02;
		const int steps = 150;
		const bool include_initial = true;

		const auto x0 = aerosyssim::sim::make_platform1d_state(-10.0, -0.75);
		auto p = aerosyssim::sim::make_platform1d_params_default();
		p.g = 9.80665;
		const auto u = aerosyssim::sim::make_platform1d_control(p.g);

		const auto tr = aerosyssim::sim::run_platform1d_fixed_step_constant_control(
			t0, dt, steps, include_initial, x0, u, p
		);

		const double tf = dt * static_cast<double>(steps);
		const double d_expected = x0.d + x0.vd * tf;
		const double vd_expected = x0.vd;

		const double d_final = tr.x.back()[0];
		const double vd_final = tr.x.back()[1];

		if (!near(d_final, d_expected, 1e-12, 1e-12)) return fail("final d does not match hover analytic solution");
		if (!near(vd_final, vd_expected, 1e-12, 1e-12)) return fail("final vd does not match hover analytic solution");
	}

	return 0;
}
