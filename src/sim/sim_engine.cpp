#include "aerosyssim/sim/sim_engine.hpp"

#include "aerosyssim/sim/attitude_stepper.hpp"
#include "aerosyssim/sim/pack.hpp"

namespace aerosyssim::sim {

SimTraceAttitude run_attitude_fixed_step(
		const SimConfigFixedStep& cfg,
		const AttitudeState& x0,
		const AttitudeControl& u,
		const RigidBodyParams& p) {

	const AttitudeControlFn u_of_t_x = [u](double, const AttitudeState&) { return u; };
	return run_attitude_fixed_step(cfg, x0, u_of_t_x, p);
}

SimTraceAttitude run_attitude_fixed_step(
		const SimConfigFixedStep& cfg,
		const AttitudeState& x0,
		const AttitudeControlFn& u_of_t_x,
		const RigidBodyParams& p) {

	SimTraceAttitude out;

	const std::size_t n_samples = cfg.include_initial ? (cfg.num_steps + 1) : cfg.num_steps;
	out.t.reserve(n_samples);
	out.x.reserve(n_samples);

	double t = cfg.t0;
	AttitudeState x = x0;

	if (cfg.include_initial) {
		out.t.push_back(t);
		out.x.push_back(pack_attitude_state(normalize_attitude_state(x)));
	}

	for (std::size_t k = 0; k < cfg.num_steps; ++k) {
		const AttitudeControl uk = u_of_t_x(t, x);
		x = step_attitude_rk4(t, cfg.dt, x, uk, p);
		t += cfg.dt;
		out.t.push_back(t);
		out.x.push_back(pack_attitude_state(x));
	}

	return out;
}

} // namespace aerosyssim::sim
