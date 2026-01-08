#include <iostream>
#include <cmath>
#include <array>

#include "aerosyssim/integration/rk4.hpp"

namespace {
	
bool approx(double a, double b, double tol) {
	return std::fabs(a - b) <= tol;
}

} // namespace

int main() {
	const double t0 = 0.0;
	const double dt = 0.01;
	const double tol = 1e-10;

	const std::array<double, 1> x0 = {1.0};

	// RHS for dx/dt = x
	auto rhs = [](double /*t*/, const std::array<double, 1>& x) -> std::array<double, 1> {
		return { x[0] };
	};

	const std::array<double, 1> x1 = aerosyssim::integration::rk4_step<1>(rhs, t0, dt, x0);

	// Analytic solution after one step
	const double expected = std::exp(dt) * x0[0];

	const double err = std::fabs(x1[0] - expected);
	if(!approx(x1[0], expected, tol)) {
		std::cerr << "test_integrator_rk4: FAIL\n"
				  << " dt = " << dt << "\n"
				  << " got = " << x1[0] << "\n"
				  << " expected = " << expected << "\n"
				  << " abs_err = " << err << "\n"
				  << " tol = " << tol << "\n";
		return 1;
	}

	return 0;
} 
