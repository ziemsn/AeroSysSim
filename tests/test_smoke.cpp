#include <iostream>
#include <cmath>
#include <array>

#include "aerosyssim/math/quat.hpp"

namespace {

	bool approx(double a, double b, double tol) { 
		return std::fabs(a - b) <= tol;
	}

	double norm_wxyz(const std::array<double, 4>& q) {
		return std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	}
}

int main() { 
	const double tol = 1e-12;

	// 1) Known non-unit quaternion -> normalize -> norm(result) ~ 1.0
	{
		const std::array<double, 4> q_in = {2.0, 3.0, 4.0, 5.0};

		const std::array<double, 4> q_out = aerosyssim::math::normalize_quaternion_wxyz(q_in);

		const double n = norm_wxyz(q_out);
		if (!approx(n, 1.0, tol)) {
			std::cerr << "test_smoke: FAIL: normalized quaterion norm is " << n
				<< ", expected 1.o within tol=" << tol << "\n";
			return 1;
		}
	}
	
	{
		const std::array<double, 4> q_zero = {00.0, 0.0, 0.0, 0.0};

		const std::array<double, 4> q_out = aerosyssim::math::normalize_quaternion_wxyz(q_zero);
		const std::array<double, 4> q_id = {1.0, 0.0, 0.0, 0.0};

		for (std::size_t i = 0; i < 4; ++i) {
			if (!approx(q_out[i], q_id[i], tol)) {
				std::cerr << "test_smoke: FAIL: zero-input normalization mismatch at index " << i
					<< ": got " << q_out[i] << ", expected " << q_id[i]
					<< " within tol=" << tol << "\n";
				return 1;
			}
		}
	}

	std::cout <<"test_smoke: OK\n";
	return 0;
}

