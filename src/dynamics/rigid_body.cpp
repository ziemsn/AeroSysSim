#include "aerosyssim/dynamics/rigid_body.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace aerosyssim::dynamics {

namespace{

math::Vec3 cross(const math::Vec3& a, const math::Vec3& b) {
	
	return math::Vec3{
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]
	};
}

math::Vec3 mat3_mul_vec3(const math::Mat3& A, const math::Vec3& v) {
	// Row-major A
	return math::Vec3{
		A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
		A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
		A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
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

math::Mat3 diag_mat3(const math::Vec3& d) {
	return math::Mat3{
		d[0], 0.0, 0.0,
		0.0, d[1], 0.0,
		0.0, 0.0, d[2]
	};
}

bool solve3x3(const math::Mat3& A, const math::Vec3& b, math::Vec3& x) {
	// Gaussian elimination with partial pivoting on augmented matrix [A|b]
	double m[3][4] = {
		{A[0], A[1], A[2], b[0]},
		{A[3], A[4], A[5], b[1]},
		{A[6], A[7], A[8], b[2]},
	};

	const double eps = 1e-14;
	x = math::Vec3{0.0, 0.0, 0.0};

	for (int col = 0; col < 3; ++col) {
		// Find pivot row
		int piv = col;
		double best = std::fabs(m[col][col]);
		for (int r = col + 1; r < 3; ++r) {
			const double v = std::fabs(m[r][col]);
			if (v > best) {
				best = v;
				piv = r;
			}
		}
		if (best < eps) {
			return false;
		}
		if (piv != col) {
			for (int c = col; c < 4; ++c) {
				std::swap(m[col][c], m[piv][c]);
			}
		}

		// Normalize pivot row
		const double inv_p = 1.0 / m[col][col];
		for (int c = col; c < 4; ++c) {
			m[col][c] *= inv_p;
		}

		// Eliminate below
		for (int r = col + 1; r < 3; ++r) {
			const double f = m[r][col];
			for (int c = col; c < 4; ++c) {
				m[r][c] -= f * m[col][c];
			}
		}
	}

	// Back substitution (pivots are 1.o due to normalization)
	for (int r = 2; r >= 0; --r) {
		double s = m[r][3];
		for (int c = r + 1; c < 3; ++c) {
			s -= m[r][c] * x[static_cast<std::size_t>(c)];
		}
		x[static_cast<std::size_t>(r)] = s;
	}

	return true;
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
	//
	// If p.inertia_body is provided, use full 3x3 inertia.
	// Otherwise, use the diagonal inertia fast path.
	
	math::Vec3 wdot{0.0, 0.0, 0.0};

	if (p.inertia_body.has_value()) {
		const math::Mat3& I = *p.inertia_body; // row-major
		const math::Vec3 Iw = mat3_mul_vec3(I, w);
		const math::Vec3 wxIw = cross(w, Iw);
		const math::Vec3 rhs_w {
			u.torque_body[0] - wxIw[0],
			u.torque_body[1] - wxIw[1],
			u.torque_body[2] - wxIw[2]
		};

		math::Vec3 sol;
		if (solve3x3(I, rhs_w, sol)) {
			wdot = sol;
		} else {
			// Singular/ill-conditioned inertia: return zeros deterministically.
			wdot = math::Vec3{0.0, 0.0, 0.0};
		}
	} else {
		const math::Vec3 Iw = diag_mul(p.inertia_body_diag, w);
		const math::Vec3 wxIw = cross(w, Iw);
		const math::Vec3 rhs_w{
			u.torque_body[0] - wxIw[0],
			u.torque_body[1] - wxIw[1],
			u.torque_body[2] - wxIw[2]
		};
		wdot = diag_inv_mul(p.inertia_body_diag, rhs_w);
	}

	//qdot = 0.5 * (q x omega)
	return PackedAttitudeState{
		0.5 * qdot[0], 0.5 * qdot[1], 0.5 * qdot[2], 0.5 * qdot[3],
		wdot[0], wdot[1], wdot[2]
	};
}

} // namespace aerosyssim::dynamics
