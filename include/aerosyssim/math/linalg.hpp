#pragma once

#include <array>

namespace aerosyssim::math {
	
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>; // row-major: m[r*3 + c]

// Returns the 3x3 skew-symmetric matrix v[x] such that:
//	skew3(v) * w) == (v x w)
Mat3 skew3(const Vec3& v);

} // namespace aerosyssim::math
