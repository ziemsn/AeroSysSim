#pragma once

#include "aerosyssim/math/linalg.hpp"
#include "aerosyssim/math/quat.hpp"

namespace aerosyssim::sim {

// All quantities are in SI units unless otherwise specified

struct AttitudeState {
	math::Quat q_wxyz;	// unit quaternion, body-to-inertial by convention
	math::Vec3 w_body;	// angular veloctiy in body frame, rad/s

};

struct AttitudeControl {
	math::Vec3 torque_body; // body-frame torque, N-m
};

struct RigidBodyParams {
	math::Vec3 inertia_body_diag; // diagonal inertia in body frame, kg-m^2
};

inline constexpr math::Quat kIdentityQuatWxyz{1.0, 0.0, 0.0, 0.0};
inline constexpr math::Vec3 kZeroVec3{0.0, 0.0, 0.0};

inline AttitudeState make_attitude_state_identity() {
	return AttitudeState{kIdentityQuatWxyz, kZeroVec3};
}

inline AttitudeControl make_attitude_control_zero() {
	return AttitudeControl{kZeroVec3};
}

} // namespace aerosyssim::sim
