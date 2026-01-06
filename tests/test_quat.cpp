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

	bool approx_quat_wxyz(const std::array<double, 4>& a,
						  const std::array<double, 4>& b,
						  double tol) {
		for (std::size_t i = 0; i < 4; i++) {
			if (!approx(a[i], b[i], tol)) {
				return false;
			}
		}
		return true;
	}
}

int main() { 
	const double tol = 1e-12;

	// Case 1: Known non-unit quaternion -> normalize -> norm(result) ~ 1.0
	{
		const std::array<double, 4> q_in = {2.0, 3.0, 4.0, 5.0};

		const std::array<double, 4> q_out = aerosyssim::math::normalize_quaternion_wxyz(q_in);

		const double n = norm_wxyz(q_out);
		if (!approx(n, 1.0, tol)) {
			std::cerr << "test_quat: FAIL: normalized quaterion norm is " << n
				<< ", expected 1.o within tol=" << tol << "\n";
			return 1;
		}
	}

	// Case 2: Zero quaternion input -> avoid divide-by-zero -> return identity quaternion {1,0,0,0}
	{
		const std::array<double, 4> q_zero = {0.0, 0.0, 0.0, 0.0};

		const std::array<double, 4> q_out = aerosyssim::math::normalize_quaternion_wxyz(q_zero);
		const std::array<double, 4> q_id = {1.0, 0.0, 0.0, 0.0};

		for (std::size_t i = 0; i < 4; ++i) {
			if (!approx(q_out[i], q_id[i], tol)) {
				std::cerr << "test_quat: FAIL: zero-input normalization mismatch at index " << i
					<< ": got " << q_out[i] << ", expected " << q_id[i]
					<< " within tol=" << tol << "\n";
				return 1;
			}
		}
	}
	
	// Case 3: Quaternion multiplication identity: q x I = q and I x q = q
	{
		const std::array<double, 4> q =	{0.5, -1.0, 2.0, -3.0};
		const std::array<double, 4> q_id = {1.0, 0.0, 0.0, 0.0};

		const auto q_right = aerosyssim::math::multiply_quaternion_wxyz(q, q_id);
		if (!approx_quat_wxyz(q_right, q, tol)){
			std::cerr << "test_quat: FAIL: q x I != q\n";
			return 1;
		}

		const auto q_left = aerosyssim::math::multiply_quaternion_wxyz(q_id, q);
		if (!approx_quat_wxyz(q_left, q, tol)){
			std::cerr << "test_quat: FAIL: I x q != q\n";
			return 1;
		}
	}

	// Case 4: Basis algebra (Hamilton product) in wxyz order:
	// i x j = k, j x i = -k, j x k = -i, k x i= j, i x k = -j
	{
		const std::array<double, 4> qi  = {0.0, 1.0, 0.0, 0.0};
		const std::array<double, 4> qj  = {0.0, 0.0, 1.0, 0.0};
		const std::array<double, 4> qk  = {0.0, 0.0, 0.0, 1.0};
		const std::array<double, 4> qmi = {0.0, -1.0, 0.0, 0.0};
		const std::array<double, 4> qmj = {0.0, 0.0, -1.0, 0.0};
		const std::array<double, 4> qmk = {0.0, 0.0, 0.0, -1.0};
		
		if (!approx_quat_wxyz(aerosyssim::math::multiply_quaternion_wxyz(qi, qj), qk, tol)) {
			std::cerr << "test_quat: FAIL: i x j != k\n";
			return 1;
		}

		if (!approx_quat_wxyz(aerosyssim::math::multiply_quaternion_wxyz(qj, qi), qmk, tol)) {
			std::cerr << "test_quat: FAIL: j x i != -k\n";
			return 1;
		}
		
		if (!approx_quat_wxyz(aerosyssim::math::multiply_quaternion_wxyz(qj, qk), qi, tol)) {
			std::cerr << "test_quat: FAIL: j x k != i\n";
			return 1;
		}

		if (!approx_quat_wxyz(aerosyssim::math::multiply_quaternion_wxyz(qk, qj), qmi, tol)) {
			std::cerr << "test_quat: FAIL: k x j != -i\n";
			return 1;
		}
		
		if (!approx_quat_wxyz(aerosyssim::math::multiply_quaternion_wxyz(qk, qi), qj, tol)) {
			std::cerr << "test_quat: FAIL: k x i != j\n";
			return 1;
		}

		if (!approx_quat_wxyz(aerosyssim::math::multiply_quaternion_wxyz(qi, qk), qmj, tol)) {
			std::cerr << "test_quat: FAIL: i x k != -j\n";
			return 1;
		}
	}

	// Case 5: Unit quaternions multiply to a unit quaternion (within tolerance)
	{
		const std::array<double, 4> a_raw = {2.0, 3.0, 4.0, 5.0};
		const std::array<double, 4> b_raw = {-1.0, 0.25, 0.5, -2.0};

		const auto a = aerosyssim::math::normalize_quaternion_wxyz(a_raw);
		const auto b = aerosyssim::math::normalize_quaternion_wxyz(b_raw);
		const auto c = aerosyssim::math::multiply_quaternion_wxyz(a, b);

		const double n = norm_wxyz(c);
		if (!approx(n, 1.0, 50.0*tol)) { 
			std::cerr << "test_quat: FAIL: norm (a x b) is " << n
					  << ", expected ~1 within tol=" << 50.0*tol << "\n";
			return 1;
		}
	}

	// Case 6: Associativity: (a x b) x c == a x (b x c)
	{
		const auto a = aerosyssim::math::normalize_quaternion_wxyz({0.9, 0.1, -0.3, 0.2});
		const auto b = aerosyssim::math::normalize_quaternion_wxyz({0.7, -0.2, 0.4, 0.1});
		const auto c = aerosyssim::math::normalize_quaternion_wxyz({0.6, 0.3, 0.2, -0.1});

		const auto left = aerosyssim::math::multiply_quaternion_wxyz(
				aerosyssim::math::multiply_quaternion_wxyz(a, b), c);
		const auto right = aerosyssim::math::multiply_quaternion_wxyz(
				a, aerosyssim::math::multiply_quaternion_wxyz(b, c));

		if (!approx_quat_wxyz(left, right, 50.0*tol)) {
			std::cerr << "test_quat: FAIL: associativity check failed \n";
			return 1;
		}
	}

	// Case 7: Non-commutativity: a xb != b x a (for a general choice)
	{
		const auto a = aerosyssim::math::normalize_quaternion_wxyz({0.9, 0.1, -0.3, 0.2});
		const auto b = aerosyssim::math::normalize_quaternion_wxyz({0.7, -0.2, 0.4, 0.1});
		
		const auto ab = aerosyssim::math::multiply_quaternion_wxyz(a, b);
		const auto ba = aerosyssim::math::multiply_quaternion_wxyz(b, a);

		if (approx_quat_wxyz(ab, ba, 50.0*tol)) {
			std::cerr << "test_quat: FAIL: expected non-commutativity but ab ~= ba \n";
			return 1;
		}
	}

	std::cout <<"test_quat: OK\n";
	return 0;
}

