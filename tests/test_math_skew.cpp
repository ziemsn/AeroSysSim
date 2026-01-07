#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>

#include "aerosyssim/math/linalg.hpp"

namespace {

bool approx(double a, double b, double tol) {
	return std::fabs(a - b) <= tol;
}

bool approx_vec3(const aerosyssim::math::Vec3& a,
				 const aerosyssim::math::Vec3& b,
				 double tol) {

	for (std::size_t i = 0; i < 3; ++i) {
		if (!approx(a[i], b[i], tol)) {
			return false;
		}
	}
	return true;
}

aerosyssim::math::Vec3 cross3(const aerosyssim::math::Vec3& v,
							  const aerosyssim::math::Vec3& w) {
	return aerosyssim::math::Vec3{
		v[1]*w[2] - v[2]*w[1],
		v[2]*w[0] - v[0]*w[2],
		v[0]*w[1] - v[1]*w[0]
	};
}

aerosyssim::math::Vec3 mat3_mul_vec3(const aerosyssim::math::Mat3& M,
									 const aerosyssim::math::Vec3& x) {
	aerosyssim::math::Vec3 y{0.0, 0.0, 0.0};
	for (std::size_t r = 0; r < 3; ++r) {
		y[r] = M[r*3 + 0]*x[0] + M[r*3 + 1]*x[1] + M[r*3 + 2]*x[2];
	}
	return y;
}
} // namespace

int main() {
	const double tol = 1e-12;

	// Case 1: Skew symmetry: S + S^T = 0
	{
		const aerosyssim::math::Vec3 v{1.2, -3.4, 5.6};
		const auto S = aerosyssim::math::skew3(v);

		for (std::size_t r= 0; r < 3; ++r) {
			for (std::size_t c = 0; c < 3; ++c) {
				const double sum = S[r*3 + c] + S[c*3 + r];
				if(!approx(sum, 0.0, tol)) {
					std::cerr << "test_math_skew: FAIL: S + S^T != 0 at ("
							  << r << "," << c <<"): " << sum
							  << " (tol=" << tol << ")\n";
					return 1 ;
				}
			}
		}
	}
	
	// Case 2: Cross-product equivalence: skew(v)*w == v x w
	{
		const struct {
			aerosyssim::math::Vec3 v;
			aerosyssim::math::Vec3 w;
		} cases[] = {
			{ {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0} },
			{ {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0} },
			{ {-2.0, 1.5, 0.5}, {0.25, -4.0, 2.0} }
		};

		for (const auto& tc : cases) {
			const auto S = aerosyssim::math::skew3(tc.v);
			const auto Sw = mat3_mul_vec3(S, tc.w);
			const auto vxw = cross3(tc.v, tc.w);

			if (!approx_vec3(Sw, vxw, tol)) {
				std::cerr << "test_math_skew: FAIL: skew(v)*w != v x w\n"
						  << " v = [" << tc.v[0] << ", " << tc.v[1] << ", " << tc.v[2] << "]\n"
						  << " w = [" << tc.w[0] << ", " << tc.w[1] << ", " << tc.w[2] << "]\n"
						  << " skew(v)*w = [" << Sw[0] << ", "  << Sw[1] << ", " << Sw[2] << "]\n"
						  << " v x w     = [" << vxw[0] << ", "  << vxw[1] << ", " << vxw[2] << "]\n"
						  << " tol=" << tol << "\n";
				return 1;
			}
		}
	}

	std::cout << "test_math_skew: OK\n";
	return 0;

}

