#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "aerosyssim/sim/types.hpp"

namespace aerosyssim::sim {

// Control callback for time-varying torque policies
// Evaluated at the start of each integration step with (t, x)
using AttitudeControlFn = std::function<AttitudeControl(double t, const AttitudeState& x)>;

// Piecewise-constant body torque schedule.
// Invariant: torque_body.size() == t_breaks.size() + 1 and t_breaks strictly increasing
struct PiecewiseConstantTorqueSchedule {
	std::vector<double> t_breaks;
	std::vector<aerosyssim::math::Vec3> torque_body;

	bool is_valid() const {
		if (torque_body.empty()) {
			return false;
		}
		if (torque_body.size() != t_breaks.size() + 1) {
			return false;
		}
		for (std::size_t i = 1; i < t_breaks.size(); ++i) {
			if (!(t_breaks[i] > t_breaks[i - 1])) {
				return false;
			}
		}
		return true;
	}

	AttitudeControl operator()(double t, const AttitudeState&) const {
		if (torque_body.empty()) {
			return AttitudeControl{aerosyssim::math::Vec3{0.0, 0.0, 0.0}};
		}
		std::size_t idx = 0;
		while (idx < t_breaks.size() && t >= t_breaks[idx]) {
			++idx;
		}
		if (idx >= torque_body.size()) {
			idx = torque_body.size() - 1;
		}
		return AttitudeControl{torque_body[idx]};
	}
};

} // namespace aerosyssim::sim
