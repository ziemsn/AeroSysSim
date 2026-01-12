#include "aerosyssim/dynamics/rigid_body.hpp"

#include <array>
#include <cmath>
#include <iostream>

namespace {
bool near(double a, double b, double tol) {
	return std::fabs(a - b) <= tol;
}

template <std::size_t N>
bool near_array(const std::array<double, N>& a, const std::array<double, N>& b, double tol) {
	for (std::size_t i = 0; i < N; ++i) {
		if (!near(a[i], b[i], tol)) {
			return false;
		}
	}
	return true;
}

int fail(const char* msg) {
	std::cerr << "test_rigid_body_rhs: FAIL: " << msg << "\n";
	return 1;
}

} //namespace

int main() {
	using aerosyssim::dynamics::PackedAttitudeState;
	using aerosyssim::dynamics::AttitudeControl;
	using aerosyssim::dynamics::RigidBodyParams;

	const double tol = 1e-12;

	// Case 1: q = identity, w aligned with principle axis, tau = 0
	{
		const PackedAttitudeState x{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
		const AttitudeControl u{{0.0, 0.0,  0.0}};
		const RigidBodyParams p{{2.0, 3.0, 4.0}};

		const auto dx = aerosyssim::dynamics::rigid_body_attitude_rhs_packed(0.0, x, u, p);
		const PackedAttitudeState expected{0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0};

		if (!near_array(dx, expected, tol)) {
			return fail("principal-axis free rotation case mismatch");
		}
	}

	// Case 2: general w and nonzero torque with diagonal inertia
	{
		const PackedAttitudeState x{1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0};
		const AttitudeControl u{{10.0, 20.0,  30.0}};
		const RigidBodyParams p{{2.0, 3.0, 4.0}};

		const auto dx = aerosyssim::dynamics::rigid_body_attitude_rhs_packed(0.0, x, u, p);

		if (!near(dx[4], 2.0, tol)) {
			return fail("wdot_x mismatch");
		}

		if (!near(dx[5], 26.0 / 3.0, tol)) {
			return fail("wdot_y mismatch");
		}

		if (!near(dx[6], 7.0, tol)) {
			return fail("wdot_z mismatch");
		}
	}

	// Case 3: gernal w and nonzero torque with full (non-diagonal) inertia.
	// Uses a matrix with a known closed-form inverse to avoid relying on the same solver.
	//
	// I = [ 2 1 0
	//       1 2 0 
	//       0 0 3 ]
	//
	// I^{-1} = [ 2/3 -1/3 0
	//           -1/3 2/3  0
	//             0    0 1/3 ]
	{
		const PackedAttitudeState x{1.0, 0.0, 0.0, 0.0, 0.4, -0.5, 0.6};
		const AttitudeControl u{{0.7, -0.2, 0.1}};

		RigidBodyParams p{{2.0, 2.0, 3.0}}; // diag is present but should be ignored when inertia_body is set
		p.inertia_body = aerosyssim::math::Mat3{
			2.0, 1.0, 0.0,
			1.0, 2.0, 0.0,
			0.0, 0.0, 3.0
		};

		const auto dx = aerosyssim::dynamics::rigid_body_attitude_rhs_packed(0.0, x, u, p);

		// Expected wdot computed analytically:
		// rhs = tau - w x (I w), then wdot = I^{-1} rhs.
		//
		// For this case:
		// wdot_x = 107/150
		// wdot_y = -14/75
		// wdot_z = 19/300
		if (!near(dx[4], 107.0 / 150.0, tol)) {
			return fail("full-inertia wdot_x mismatch");
		}
		if (!near(dx[5], -14.0 / 75.0, tol)) {
			return fail("full-inertia wdot_y mismatch");
		}
		if (!near(dx[6], 19.0 / 300.0, tol)) {
			return fail("full-inertia wdot_z mismatch");
		}
	}

	return 0;
}
