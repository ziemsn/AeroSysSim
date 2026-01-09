#include "aerosyssim/sim/attitude_stepper.hpp"
#include "aerosyssim/sim/pack.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

namespace {

bool near(double a, double b, double tol) {
	return std::fabs(a - b) <= tol;
}

int fail(const char* msg) {
	std::cerr << "test_attitude_stepper: FAIL: " << msg << "\n";
	return 1;
}

} // namespace

int main() {
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::AttitudeControl;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::pack_attitude_state;

	// Case 1: principal-axis free rotation
	{
		AttitudeState x;
		x.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
		x.w_body = aerosyssim::math::Vec3{1.0, 0.0, 0.0};

		const AttitudeControl u{{0.0, 0.0, 0.0}};
		const RigidBodyParams p{{2.0, 3.0, 4.0}};

		const double t = 0.0;
		const double dt = 0.01;

		const auto x1 = aerosyssim::sim::step_attitude_rk4(t, dt, x, u, p);

		// w should remain constant for this case (wdot is exactly zero)
		
		if (x1.w_body[0] != 1.0 || x1.w_body[1] != 0.0 || x1.w_body[2] != 0.0) {
			return fail("principal-axis case should keep w constant");
		}

		const double half = 0.5 * dt;
		const aerosyssim::math::Quat q_expected{std::cos(half), std::sin(half), 0.0, 0.0};
		const double tol_q = 1e-10;

		if (!near(x1.q_wxyz[0], q_expected[0], tol_q) ||
			!near(x1.q_wxyz[1], q_expected[1], tol_q) ||
			!near(x1.q_wxyz[2], 0.0, tol_q) ||
			!near(x1.q_wxyz[3], 0.0, tol_q)) {
			
			return fail("principal-axis quaternion mismatch vs analytic");
		}

		if (!near(aerosyssim::math::quaternion_norm(x1.q_wxyz), 1.0, 1e-12)) {
			return fail("quaternion shuold be unit length after step");
		}
	}

	// Case 2: non-unit input quaternion should be normalized deterministically
	{
		AttitudeState x;
		x.q_wxyz = aerosyssim::math::Quat{2.0, 0.0, 0.0, 0.0};
		x.w_body = aerosyssim::math::Vec3{0.0, 0.0, 0.0};

		const AttitudeControl u{{0.0, 0.0, 0.0}};
		const RigidBodyParams p{{2.0, 3.0, 4.0}};

		const auto x1 = aerosyssim::sim::step_attitude_rk4(0.0, 0.1, x, u, p);
		if (x1.q_wxyz[0] != 1.0 || x1.q_wxyz[1] != 0.0 || x1.q_wxyz[2] != 0.0 || x1.q_wxyz[3] != 0.0 ) {
			return fail("normalized identity quaternion should have norm exactly 1");
		}
	}

	// Case 3: determinism, byte identical packed output for identical inputs
	{
		AttitudeState x;
		x.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
		x.w_body = aerosyssim::math::Vec3{0.3, -0.2, 0.1};

		const AttitudeControl u{{0.0, 0.0, 0.0}};
		const RigidBodyParams p{{2.0, 3.0, 4.0}};

		const auto a = aerosyssim::sim::step_attitude_rk4(0.0, 0.01, x, u, p);
		const auto b = aerosyssim::sim::step_attitude_rk4(0.0, 0.01, x, u, p);

		const auto pa = pack_attitude_state(a);
		const auto pb = pack_attitude_state(b);

		if (std::memcmp(pa.data(), pb.data(), sizeof(double) * pa.size()) != 0) {
			return fail("stepper results should be byte-identical for identical inputs");
		}
	}

	return 0;
}
