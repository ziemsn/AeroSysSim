#include "aerosyssim/dynamics/platform_1d.hpp"

#include <iostream>

namespace {

int fail(const char* msg) {
	std::cerr << "test_platform_1d_rhs: FAIL: " << msg << "\n";
	return 1;
}

} // namespace

int main() {
	aerosyssim::sim::Platform1DParams p = aerosyssim::sim::make_platform1d_params_default();
	p.g = 10.0;

	const aerosyssim::sim::Platform1DState x{1.0, 2.0}; // d=1, vd=2

	// No thrust: vd_dot = g
	{
		const aerosyssim::sim::Platform1DControl u{0.0};
		const auto xdot = aerosyssim::dynamics::platform1d_rhs(x, u, p);
		if (xdot.d != 2.0) return fail("d_dot should equal vd");
		if (xdot.vd != 10.0) return fail("vd_dot should equal g when a_cmd=0");
	}

	// Hover thrust: vd_dot = 0 when a_cmd = g.
	{
		const aerosyssim::sim::Platform1DControl u{10.0};
		const auto xdot = aerosyssim::dynamics::platform1d_rhs(x, u, p);
		if (xdot.vd!= 0.0) return fail("vd_dot should be 0 when a_cmd=g");
	}

	// Excess thrust: vd_dot negative when a_cmd > g (accelearte Up, whish is -Down)
	{
		const aerosyssim::sim::Platform1DControl u{12.0};
		const auto xdot = aerosyssim::dynamics::platform1d_rhs(x, u, p);
		if (!(xdot.vd < 0.0)) return fail("vd_dot should be negative when a_cmd > g");
	}

	return 0;
}
