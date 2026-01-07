#include "aerosyssim/math/linalg.hpp"

namespace aerosyssim::math {
	Mat3 skew3(const Vec3& v) {
		const double vx = v[0];
		const double vy = v[1];
		const double vz = v[2];

		return Mat3{
			0.0, -vz, vy,
			vz, 0.0, -vx, 
			-vy, vx, 0.0
		};
	}
}
