#include "aerosyssim/sim/types.hpp"

#include <iostream>

namespace {

int fail (const char* msg) {
	std::cerr << "test_types: FAIL: " << msg << "\n";
	return 1;
}

} // namespace

int main() {

	const auto s = aerosyssim::sim::make_attitude_state_identity();
	if (s.q_wxyz[0] != 1.0 || s.q_wxyz[1] != 0.0 || s.q_wxyz[2] != 0.0 || s.q_wxyz[3] != 0.0) {
		return fail("identity quaternion should be {1, 0, 0, 0}");
	}	

	if (s.w_body[0] != 0.0 || s.w_body[1] != 0.0 || s.w_body[2] != 0.0) {
		return fail("default anguler velocity should be zero");
	}

	const auto u = aerosyssim::sim::make_attitude_control_zero();
	if (u.torque_body[0] != 0.0 || u.torque_body[1] != 0.0 || u.torque_body[2] != 0.0) {
		return fail("default torque should be zero");
	}

	aerosyssim::sim::RigidBodyParams p{{2.0, 3.0, 4.0}};
	if (p.inertia_body_diag[0] <= 0.0 || p.inertia_body_diag[1] <= 0.0 ||p.inertia_body_diag[2] <= 0.0) {
		return fail("inertia diagonal must be positive");
	}

	return 0;
}
