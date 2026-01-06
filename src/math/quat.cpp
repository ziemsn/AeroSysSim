#include <array>
#include <cmath>
#include <iostream>

#include "aerosyssim/math/quat.hpp"

namespace aerosyssim::math {

	std::array<double, 4> normalize_quaternion_wxyz(const std::array<double, 4>& q) {
		constexpr double epsilon = 1e-12;

		const double norm_quat  = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

		if (norm_quat <= epsilon) {
			std::cout << "Quat norm vanishingly small" << std::endl;
			return {1.0, 0.0, 0.0, 0.0};
		} 

		return {
			q[0] / norm_quat,
			q[1] / norm_quat,
			q[2] / norm_quat,
			q[3] / norm_quat
		};
	}
}
