#include <iostream>
#include <iomanip>

#include "aerosyssim/sim/sim_engine.hpp"

int main() {
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::AttitudeControl;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::SimConfigFixedStep;

	AttitudeState x0;
	x0.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
	x0.w_body = aerosyssim::math::Vec3{0.3, -0.2, 0.1};

	const AttitudeControl u{{0.0, 0.0, 0.0}};
	const RigidBodyParams p{{2.0, 3.0, 4.0}};

	SimConfigFixedStep cfg;
	cfg.t0 = 0.0;
	cfg.dt = 0.01;
	cfg.num_steps = 50;
	cfg.include_initial = true;

	const auto trace = aerosyssim::sim::run_attitude_fixed_step(cfg, x0, u, p);

	// Stable CSV formatting
	std::cout.setf(std::ios::scientific);
	std::cout << std::setprecision(17);

	std::cout << "t,qw,qx,qy,qz,wx,wy,wz\n";
	for (std::size_t i = 0; i < trace.x.size(); ++i) {
		const auto& xi = trace.x[i];
		std::cout << trace.t[i] << ","
				  << xi[0] << "," << xi[1] << "," << xi[2] << "," << xi[3] << ","
				  << xi[4] << "," << xi[5] << "," << xi[6] << "\n";
	}

	return 0;

}
