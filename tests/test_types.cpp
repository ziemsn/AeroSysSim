#include <iostream>

#include "aerosyssim/sim/types.hpp"
#include "aerosyssim/sim/platform_types.hpp"\


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

	const auto x1 = aerosyssim::sim::make_platform1d_state(10.0, -2.0);
	if (x1.d != 10.0 || x1.vd != -2.0) {
		return fail("Platform1DState should presever constructor values");
	}

	const auto u1 = aerosyssim::sim::make_platform1d_control(1.5);
	if (u1.a_cmd != 1.5) return fail("Platform1DControl a_cmd should presever constructor value");

	const auto p1 = aerosyssim::sim::make_platform1d_params_default();
	if (p1.g != 9.80665) return fail("Platform1DParams default g should be 9.80665");

	if (!(p1.a_cmd_min < p1.a_cmd_max)){
		return fail("Platform1DParams default control bounds must satisfy min < max");
	}
	if (p1.d_ground != 0.0) return fail("Platform1DParams default d_ground should be 0.0");

	const auto x2 = aerosyssim::sim::make_platform2d_state(1.0, 2.0, 3.0, 4.0);
	if (x2.n != 1.0 || x2.d != 2.0 || x2.vn != 3.0 || x2.vd!= 4.0) {
		return fail("Platform2DState should presever constructor values");
	}

	const auto u2 = aerosyssim::sim::make_platform2d_control(0.1, -0.2);
	if (u2.an_cmd != 0.1 || u2.ad_cmd != -0.2) {
		return fail("Platform2DControl should preseve constructor values");
	}

	const auto p2 = aerosyssim::sim::make_platform2d_params_default();
	if (p2.g != 9.80665) return fail("Platform2DParams default g should be 9.80665");

	if (!(p2.an_cmd_min < p2.an_cmd_max)) {
		return fail("Platform2DParams default north control bounds must satisfy min < max");
	}
	if (!(p2.ad_cmd_min < p2.ad_cmd_max)) {
		return fail("Platform2DParams default down control bounds must satisfy min < max");
	}
	if (p2.d_ground != 0.0) return fail("Platform2DParams default d_ground should be 0.0");

	return 0;
}
