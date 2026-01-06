#pragma once

#include <array>

namespace aerosyssim::math {

// Normalize a quaternion in wxyz order
// Returns a normalized quaternion
std::array<double, 4> normalize_quaternion_wxyz(const std::array<double, 4>& q);

// Hamilton product (quaternion multiply) in wxyz order
// Returns a x b
std::array<double, 4> multiply_quaternion_wxyz(
		const std::array<double, 4>& a,
		const std::array<double, 4>& b
	);

}
