#pragma once

#include <array>

namespace aerosyssim::math {

// Normalize a quaternion in wxyz order
// Returns a normalized quaternion
std::array<double, 4> normalize_quaternion_wxyz(const std::array<double, 4>& q);

}
