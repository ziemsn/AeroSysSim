#pragma once

#include <cstddef>
#include <cmath>
#include <limits>

#include "aerosyssim/math/linalg.hpp"
#include "aerosyssim/math/quat.hpp"
#include "aerosyssim/sim/sim_engine.hpp"

namespace aerosyssim::sim {

struct TraceInvariantStats {
	// Quaternion norm stats
	double qnorm_min{std::numeric_limits<double>::infinity()};
	double qnorm_max{-std::numeric_limits<double>::infinity()};
	double qnorm_max_abs_err{0.0}; // max | ||q|| - 1 |

	// Rotational kinetic energy stats
	double energy0{0.0};
	double energy_min{std::numeric_limits<double>::infinity()};
	double energy_max{-std::numeric_limits<double>::infinity()};
	double energy_max_abs_err{0.0}; // max |E - E0|
	double energy_rel_drift{0.0};   // max |E - E0| / max(|E0|, eps)

	// Angular momentum magnitude stats (L = I w)
	double Lnorm0{0.0};
	double Lnorm_min{std::numeric_limits<double>::infinity()};
	double Lnorm_max{-std::numeric_limits<double>::infinity()};
	double Lnorm_max_abs_err{0.0}; // max |L - L0|
	double Lnorm_rel_drift{0.0};   // max |L - L0| / max(|L0|, eps)
};

namespace detail {

inline double dot3(const math::Vec3&a, const math::Vec3& b) {
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline double norm3(const math::Vec3& v) {
	return std::sqrt(dot3(v, v));
}

inline math::Vec3 diag_mul(const math::Vec3& d, const math::Vec3& v) {
	return math::Vec3{d[0]*v[0], d[1]*v[1], d[2]*v[2]};
}

inline math::Vec3 mat3_mul_vec3(const math::Mat3& A, const math::Vec3& v) {
	// Row-major A
	return math::Vec3{
		A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
		A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
		A[6] * v[0] + A[7] * v[1] + A[8] * v[2]
	};
}

inline math::Vec3 unpack_w_body(const PackedAttitudeState& x) {
	return math::Vec3{x[4], x[5], x[6]};
}

inline math::Quat unpack_q_wxyz(const PackedAttitudeState& x) {
	return math::Quat{x[0], x[1], x[2], x[3]};
}

} // namespace detail

inline math::Vec3 angular_momentum_body(const math::Vec3& w_body, const RigidBodyParams& p) {
	if (p.inertia_body.has_value()) {
		return detail::mat3_mul_vec3(*p.inertia_body, w_body);
	}
	return detail::diag_mul(p.inertia_body_diag, w_body);
}

inline double rotational_kinetic_energy(const math::Vec3& w_body, const RigidBodyParams& p) {
	const math::Vec3 L = angular_momentum_body(w_body, p); // L = I w
	return 0.5 * detail::dot3(w_body, L);
}

inline TraceInvariantStats compute_trace_invariants(const SimTraceAttitude& tr, const RigidBodyParams& p) {
	TraceInvariantStats s;
	if (tr.x.empty()) {
		return s;
	}

	const math::Vec3 w0 = detail::unpack_w_body(tr.x.front());
	s.energy0 = rotational_kinetic_energy(w0, p);
	s.Lnorm0 = detail::norm3(angular_momentum_body(w0, p));

	const double eps = 1e-14;

	for (const auto& xi : tr.x) {
		const math::Quat q = detail::unpack_q_wxyz(xi);
		const double qn = math::quaternion_norm(q);
		s.qnorm_min = std::min(s.qnorm_min, qn);
		s.qnorm_max = std::max(s.qnorm_max, qn);
		s.qnorm_max_abs_err = std::max(s.qnorm_max_abs_err, std::fabs(qn - 1.0));
		
		const math::Vec3 w = detail::unpack_w_body(xi);
		const double E = rotational_kinetic_energy(w, p);
		s.energy_min = std::min(s.energy_min, E);
		s.energy_max = std::max(s.energy_max, E);
		s.energy_max_abs_err = std::max(s.energy_max_abs_err, std::fabs(E - s.energy0));

		const double Lnorm = detail::norm3(angular_momentum_body(w, p));
		s.Lnorm_min = std::min(s.Lnorm_min, Lnorm);
		s.Lnorm_max = std::max(s.Lnorm_max, Lnorm);
		s.Lnorm_max_abs_err = std::max(s.Lnorm_max_abs_err, std::fabs(Lnorm - s.Lnorm0));
	}

	s.energy_rel_drift = s.energy_max_abs_err / std::max(std::fabs(s.energy0), eps);
	s.Lnorm_rel_drift = s.Lnorm_max_abs_err / std::max(std::fabs(s.Lnorm0), eps);
	return s;
}

} // namespace aerosyssim::sim
