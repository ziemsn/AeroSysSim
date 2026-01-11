#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <string>
#include <locale>
#include <vector>

#include "aerosyssim/sim/sim_engine.hpp"

namespace {

struct AppConfig {
	double t0 = 0.0;
	double dt = 0.01;
	std::size_t steps = 50;
	aerosyssim::math::Vec3 w0{0.3, -0.2, 0.1};
	std::string scenario = "principal_axis";
	bool w_user_set = false;
};

void print_help() {
	std::cout.imbue(std::locale::classic());
	std::cout 
		<< "sim_runner options: \n"
		<< " --dt <dt>\n"
		<< " --steps <N>\n"
		<< " --t0 <t0>\n"
		<< " --w <wx> <wy> <wz>\n"
		<< " --scenario <principal_axis|coupled_rates>\n"
		<< " --help\n";
}

bool parse_double(const std::string& s, double& out) {
	char* end = nullptr;
	out = std::strtod(s.c_str(), &end);
	return end && *end == '\0';
}

bool parse_size(const std::string& s, std::size_t& out) {
	char* end = nullptr;
	const unsigned long v = std::strtoul(s.c_str(), &end, 10);
	if (!(end && *end == '\0')) {
		return false;
	}
	out = static_cast<std::size_t>(v);
	return true;
}

enum class ParseStatus {Ok, Help, Error};

ParseStatus parse_args(int argc, char** argv, AppConfig& cfg) {
	std::vector<std::string> args;
	args.reserve(static_cast<std::size_t>(argc));
	for (int i = 1; i < argc; ++i) {
		args.emplace_back(argv[i]);
	}

	for (std::size_t i = 0; i < args.size(); ++i) {
		const std::string& a = args[i];
		if (a == "--help") {
			print_help();
			return ParseStatus::Help;
		} else if (a == "--dt") {
			if (i + 1 >= args.size() || !parse_double(args[i + 1], cfg.dt)) {
				std::cerr << "sim_runner :invalid --dt\n";
				return ParseStatus::Error;
			}
			i += 1;
		} else if (a == "--t0") {
			if (i + 1 >= args.size() || !parse_double(args[i + 1], cfg.t0)) {
				std::cerr << "sim_runner :invalid --t0\n";
				return ParseStatus::Error;
			}
			i += 1;
		} else if (a == "--steps") {
			if (i + 1 >= args.size() || !parse_size(args[i + 1], cfg.steps)) {
				std::cerr << "sim_runner :invalid --steps\n";
				return ParseStatus::Error;
			}
			i += 1;
		} else if (a == "--w") {
			double wx = 0.0, wy = 0.0, wz = 0.0;
			if (i + 3 >= args.size() || 
				!parse_double(args[i + 1], wx) ||
				!parse_double(args[i + 2], wy) ||
				!parse_double(args[i + 3], wz)) {
				std::cerr << "sim_runner :invalid --w\n";
				return ParseStatus::Error;
			}
			cfg.w0 = aerosyssim::math::Vec3{wx, wy, wz};
			cfg.w_user_set = true;
			i += 3;
		} else if (a == "--scenario") { 
			if (i + 1 >= args.size()) {
				std::cerr << "sim_runner: invalid --scenario\n";
				return ParseStatus::Error;
			}
			cfg.scenario = args[i + 1];
			i += 1;
		} else {
			std::cerr << "sim_runner: unknown option: " << a << "\n";
			return ParseStatus::Error;
		}
	}
	return ParseStatus::Ok;
}

} // namespace

int main(int argc, char** argv) {
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::AttitudeControl;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::SimConfigFixedStep;

	AppConfig app_cfg;
	const auto st = parse_args(argc, argv, app_cfg);
	if (st == ParseStatus::Help) {
		return 0;
	}
	if (st == ParseStatus::Error) {
		return 1;
	}

	// Scanerio defaults (only apply if usesr did not explicitly provide --w)
	if (app_cfg.scenario == "principal_axis") {
		// default already matches this scenario
		if (!app_cfg.w_user_set) {
			app_cfg.w0 = aerosyssim::math::Vec3{0.3, -0.2, 0.1};
		}
	} else if (app_cfg.scenario == "coupled_rates") {
		if (!app_cfg.w_user_set) {
			app_cfg.w0 = aerosyssim::math::Vec3{0.6, 0.4, -0.3};
		}
	} else {
		std::cerr << "sim_runner: unknown scenario: " << app_cfg.scenario << "\n";
		print_help();
		return 1;
	}

	AttitudeState x0;
	x0.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
	x0.w_body = app_cfg.w0;

	const AttitudeControl u{{0.0, 0.0, 0.0}};
	const RigidBodyParams p{{2.0, 3.0, 4.0}};

	SimConfigFixedStep cfg;
	cfg.t0 = app_cfg.t0;
	cfg.dt = app_cfg.dt;
	cfg.num_steps = app_cfg.steps;
	cfg.include_initial = true;

	const auto trace = aerosyssim::sim::run_attitude_fixed_step(cfg, x0, u, p);

	// Stable CSV formatting
	std::cout.imbue(std::locale::classic());
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
