#pragma once

#include "aerosyssim/dynamics/rigid_body.hpp"
#include "aerosyssim/integration/rk4.hpp"
#include "aerosyssim/sim/pack.hpp"

namespace aerosyssim::sim {

//One RK4 step for attitude dynamics with quaternion normalization
inline AttitudeState step_attitude_rk4(
		double t, 
		double dt,
		const AttitudeState& x,
		const AttitudeControl& u,
		const RigidBodyParams& p) {

	// Normalize input
	const AttitudeState x0 = normalize_attitude_state(x);
	const PackedAttitudeState xp0 = pack_attitude_state(x0);

	auto rhs = [&](double t_local, const PackedAttitudeState& xp) {
		return aerosyssim::dynamics::rigid_body_attitude_rhs_packed(t_local, xp, u, p);
	};

	const PackedAttitudeState xp1 = aerosyssim::integration::rk4_step<7>(rhs, t, dt, xp0);

	AttitudeState x1 = unpack_attitude_state(xp1);
	x1.q_wxyz = aerosyssim::math::normalize_quaternion_wxyz(x1.q_wxyz);
	return x1;
}

} // namespace aerosysssim::sim

