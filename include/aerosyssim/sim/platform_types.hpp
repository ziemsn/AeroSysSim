#pragma once

#include <array>

namespace aerosyssim::sim {

// All quantities are in SI units unless otherwise specified

// ----------------------------
// 1D vertical motion in NED (Down axis, d positive down)
// x = [d, vd]
using PackedPlatform1DState = std::array<double, 2>;

struct Platform1DState {
	double d;  // down position, [m]
	double vd; // down velocity, [m/s]
};

struct Platform1DControl {
	// commanded acceleartion along +DOWN (NED)
	double a_cmd; // [m/s^2]
};

struct Platform1DParams {
	double g; // gravity magnitude [m/s^2] (acts in +Down)

	// Optional control limits
	double a_cmd_min; // [m/s^2]
	double a_cmd_max; // [m/s^2]

	// Ground plane definition for landing tasks.
	double d_ground; // [m]
};

//------------------------
// 2D platform in NED n-d plane
// x = [n, d, vn, vd]
using PackedPlatform2DState = std::array<double, 4>;

struct Platform2DState {
	double n;   // horizontal position [m]
	double d;   // vertical position [m]
	double vn;  // horizontal velocity [m/s]
	double vd;  // vertical velocity [m/s]
};

struct Platform2DControl {
	double an_cmd; // command acceleration in North [m/s^2]
	double ad_cmd; // command acceleration in Down [m/s^2]
};

struct Platform2DParams {
	double g; // gravity magnitude [m/s^2] (acts in +Down
	
	// Optional control limits
	double an_cmd_min; // [m/s^2]
	double an_cmd_max; // [m/s^2]
	double ad_cmd_min; // [m/s^2]
	double ad_cmd_max; // [m/s^2]
	
	double d_ground; // [m]
};

inline Platform1DState make_platform1d_state(const double d0, const double vd0) {
	return Platform1DState{d0, vd0};
}

inline Platform1DControl make_platform1d_control(const double a_cmd) {
	return Platform1DControl{a_cmd};
}

inline Platform1DParams make_platform1d_params_default() {
	return Platform1DParams{
		9.80665,	// g
		-50.0,		// a_cmd_min
		50.0,		// a_cmd_max
		0.0			// d_ground
	};
}

inline Platform2DState make_platform2d_state(
		const double n0, const double d0, const double vn0, const double vd0) {
	return Platform2DState{n0, d0, vn0, vd0};
}

inline Platform2DControl make_platform2d_control(const double an_cmd, const double ad_cmd) {
	return Platform2DControl{an_cmd, ad_cmd};
}

inline Platform2DParams make_platform2d_params_default() {
	return Platform2DParams{
		9.80665,	// g
		-50.0,		// an_cmd_min
		50.0,		// an_cmd_max
		-50.0,		// ad_cmd_min
		50.0,		// ad_cmd_max
		0.0			// d_ground
	};
}

} // namespace aerosyssim::sim
