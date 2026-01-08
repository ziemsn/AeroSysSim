#pragma once

#include <array>
#include <cstddef>

namespace aerosyssim::integration {


namespace detail {

template <std::size_t N>
std::array<double, N> add(const std::array<double, N>& u,
						  const std::array<double, N>& v) {
	std::array<double, N> result;
	for (std::size_t i = 0; i < N; ++i) {
		result[i] = u[i] + v[i];
	}
	return result;
}

template <std::size_t N>
std::array<double, N> scale(double scalar,
							const std::array<double, N>& v) {
	std::array<double, N> result;
	for (std::size_t i = 0; i < N; ++i) {
		result[i] = scalar * v[i];	
	}
	return result;
}

template <std::size_t N>
std::array<double, N> axpy(double alpha,
						   const std::array<double, N>& x,
						   const std::array<double, N>& y) { 
	std::array<double, N> result;
	for (std::size_t i = 0; i < N; ++i) {
		result[i] = alpha * x[i]  + y[i];
	}
	return result;
} 

} // namespace detail 

// RK4 step for a fixed-size state vector x
// f(t, x) must return dx/dt with the same type std::array<double, N>
template <std::size_t N, typename RHS>
std::array<double, N> rk4_step(
		RHS&& f, 
		double t,
		double dt,
		const std::array<double, N>& x) {

	const double half_dt = 0.5 * dt; 

	auto k1 = f(t, x);
	auto x2 = detail::axpy(half_dt, k1, x);
	auto k2 = f(t + half_dt, x2);
	auto x3 = detail::axpy(half_dt, k2, x);
	auto k3 = f(t + half_dt, x3);
	auto x4 = detail::axpy(dt, k3, x);
	auto k4 = f(t + dt, x4);

	// x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
	auto temp = detail::add(k1, k4);
	temp = detail::add(temp, detail::scale(2.0, k2));
	temp = detail::add(temp, detail::scale(2.0, k3));
	auto x_next = detail::axpy(dt / 6.0, temp, x);

	return x_next;
}

} // namespace aerosyssim::integration


