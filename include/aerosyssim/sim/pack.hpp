#pragma once

#include "aerosyssim/sim/types.hpp"
#include "aerosyssim/math/quat.hpp"

namespace aerosyssim::sim {

inline PackedAttitudeState pack_attitude_state(const AttitudeState& s) {
	return PackedAttitudeState{
		s.q_wxyz[0], s.q_wxyz[1], s.q_wxyz[2], s.q_wxyz[3], 
		s.w_body[0], s.w_body[1], s.w_body[2]
	};
}

inline AttitudeState unpack_attitude_state(const PackedAttitudeState& x) {
	AttitudeState s;
	s.q_wxyz = math::Quat{x[0], x[1], x[2], x[3]};
	s.w_body = math::Vec3{x[4], x[5], x[6]};
	return s;
}

inline AttitudeState normalize_attitude_state(const AttitudeState& s) {
	AttitudeState out = s;
	out.q_wxyz = math::normalize_quaternion_wxyz(out.q_wxyz);
	return out;
}

} // namespace aerosyssim::sim
