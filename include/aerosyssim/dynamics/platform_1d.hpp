#pragma once

#include "aerosyssim/sim/platform_types.hpp"

namespace aerosyssim::dynamics {

// 1D NED (Down positive translational dynamics.
// Control convention: a_cmd is thrust acceleration magnitude opposing gravity
// d_dot = vd, vd_dot = g - a_cmd
aerosyssim::sim::Platform1DState platform1d_rhs(
		const aerosyssim::sim::Platform1DState& x,
		const aerosyssim::sim::Platform1DControl& u, 
		const aerosyssim::sim::Platform1DParams& p 
	);

} // namespace aerosyssim::dynamics
