#include <array>
#include <cmath>

#include "aerosyssim/math/quat.hpp"

namespace aerosyssim::math {

using Quat =std::array<double, 4>;

Quat normalize_quaternion_wxyz(const Quat& q) {
	constexpr double epsilon = 1e-12;

	const double norm_quat  = std::sqrt(
		q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
	);

	if (norm_quat <= epsilon) {
		return {1.0, 0.0, 0.0, 0.0};
	} 

	return {
		q[0] / norm_quat,
		q[1] / norm_quat,
		q[2] / norm_quat,
		q[3] / norm_quat
	};
}

Quat multiply_quaternion_wxyz(const Quat& a, const Quat& b) {

	const double aw = a[0], ax = a[1], ay = a[2], az = a[3];
	const double bw = b[0], bx = b[1], by = b[2], bz = b[3];

	return {
		aw*bw - ax*bx - ay*by - az*bz,			// w
		aw*bx + ax*bw + ay*bz - az*by,			// x
		aw*by - ax*bz + ay*bw + az*bx,			// y
		aw*bz + ax*by - ay*bx + az*bw			// z
	};
}
} // namespace aerosyssim::math
