#include "aerosyssim/dynamics/platform_1d.hpp"

namespace aerosyssim::dynamics {

aerosyssim::sim::Platform1DState platform1d_rhs(
		const aerosyssim::sim::Platform1DState& x,
		const aerosyssim::sim::Platform1DControl& u,
		const aerosyssim::sim::Platform1DParams& p
	) {
	aerosyssim::sim::Platform1DState xdot{};
	xdot.d = x.vd;
	xdot.vd = p.g - u.a_cmd;
	return xdot;
}

} // namespace aerosyssim::dynamics
