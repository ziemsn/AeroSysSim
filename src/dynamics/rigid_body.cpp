#include "aerosyssim/dynamics/rigid_body.hpp"

#include <cmath>

namespace aerosyssim::dynamics {
namespace{

math::Vec3 cross(const math::Vec3& a, const math::Vec3& b) {
	
	return math::Vec3{
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]
	};
}

math::Vec3 diag_mul(const math::Vec3& d, const math::Vec3& v) {
	return math::Vec3{d[0] * v[0], d[1] * v[1], d[2] * v[2]};
}

math::Vec3 diag_inv_mul(const math::Vec3& d, const math::Vec3& v) {
	
	const double eps = 1e-14;
	const double inv0 = (std::fabs(d[0]) > eps) ? (1.0 / d[0]) : 0.0;
	const double inv1 = (std::fabs(d[1]) > eps) ? (1.0 / d[1]) : 0.0;
	const double inv2 = (std::fabs(d[2]) > eps) ? (1.0 / d[2]) : 0.0;
	return math::Vec3{inv0 * v[0], inv1 * v[1], inv2 * v[2]};
}

} // namespace

PackedAttitudeState rigid_body_attitude_rhs_packed(
		double /*t*/,
		const PackedAttitudeState& x,
		const AttitudeControl& u,
		const RigidBodyParams& p) {

	const math::Quat q{x[0], x[1], x[2], x[3]};
	const math::Vec3 w{x[4], x[5], x[6]};

	// Quaternion kinematics
	const math::Quat omega_quat{0.0, w[0], w[1], w[2]};
	const math::Quat qdot = math::multiply_quaternion_wxyz(q, omega_quat);

	// Euler rotational dynamics for diagonal inertia:
	// I * wdot + w x (I_w) = tau
	const math::Vec3 Iw = diag_mul(p.inertia_body_diag, w);
	const math::Vec3 wxIw = cross(w, Iw);
	const math::Vec3 rhs_w{
		u.torque_body[0] - wxIw[0],
		u.torque_body[1] - wxIw[1],
		u.torque_body[2] - wxIw[2]
	};
	const math::Vec3 wdot = diag_inv_mul(p.inertia_body_diag, rhs_w);

	//qdot = 0.5 * (q x omega)
	return PackedAttitudeState{
		0.5 * qdot[0], 0.5 * qdot[1], 0.5 * qdot[2], 0.5 * qdot[3],
		wdot[0], wdot[1], wdot[2]
	};
}

} // namespace aerosyssim::dynamics
