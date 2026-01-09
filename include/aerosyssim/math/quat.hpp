#pragma once

#include <array>

namespace aerosyssim::math {

using Quat = std::array<double, 4>;

// Normalize a quaternion in wxyz order
// Returns a normalized quaternion
Quat normalize_quaternion_wxyz(const Quat& q);

// Hamilton product (quaternion multiply) in wxyz order
// Returns a x b
std::array<double, 4> multiply_quaternion_wxyz( const Quat& a, const Quat& b);

} // namespace aerosyssim::math
