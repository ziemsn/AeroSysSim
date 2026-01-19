#pragma once

#include "aerosyssim/dynamics/platform_1d.hpp"
#include "aerosyssim/sim/platform_types.hpp"

namespace aerosyssim::sim {

inline Platform1DState step_platform1d_rk4(
	const double dt,
	const Platform1DState& x,
	const Platform1DControl& u,
	const Platform1DParams& p
) {
	const auto add = [](const Platform1DState& a, const Platform1DState& b) {
		return Platform1DState{a.d + b.d, a.vd + b.vd};
	};
	const auto scale = [](const double s, const Platform1DState& a) {
		return Platform1DState{s * a.d, s * a.vd};
	};

	const Platform1DState k1 = aerosyssim::dynamics::platform1d_rhs(x, u, p);
	const Platform1DState k2 = aerosyssim::dynamics::platform1d_rhs(add(x, scale(0.5 * dt, k1)), u, p);
	const Platform1DState k3 = aerosyssim::dynamics::platform1d_rhs(add(x, scale(0.5 * dt, k2)), u, p);
	const Platform1DState k4 = aerosyssim::dynamics::platform1d_rhs(add(x, scale(dt, k3)), u, p);

	const Platform1DState incr = scale(dt / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4)));
	return add(x, incr);
}

} // namespace aerosyssim::sim
