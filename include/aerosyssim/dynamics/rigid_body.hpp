#pragma once

#include <array>

#include "aerosyssim/sim/types.hpp"

namespace aerosyssim::dynamics {

// Attitude state for integrators:
// x = [q_w, q_x, q_y, q_z, w_x, w_y, w_z]
using PackedAttitudeState = std::array<double, 7>;

using AttitudeControl = aerosyssim::sim::AttitudeControl;
using RigidBodyParams = aerosyssim::sim::RigidBodyParams;

// Rigid body attitude RHS (quaternion + body-rate) for a diagonal intertia tensor
// q_wxzy rotes body to inertial
// w_body is expressed in body frame
// qdot = 0.5 * (q x [0, w_body])
PackedAttitudeState rigid_body_attitude_rhs_packed(
		double t,
		const PackedAttitudeState& x,
		const AttitudeControl &u,
		const RigidBodyParams& p);

} // namespace aerosyssim::dynamics
