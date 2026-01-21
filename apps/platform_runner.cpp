#include "aerosyssim/sim/platform_engine_1d.hpp"

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>
#include <vector>

namespace {

struct AppConfig {
	double t0 = 0.0;
	double dt = 0.01;
	int steps = 200;
	bool include_initial = true;

	double d0 = 0.0;
	double vd0 = 0.0;

	double g = 9.80665;
	double a_cmd = 0.0;

	std::string output = "stdout";

};

int fail(const std::string& msg) {
	std::cerr << "platform_runner: ERROR: " << msg << "\n";
	return 1;
}

void print_help() {
	std::cout
		<< "platform_runner (Platform 1D NED Down axis)\n"
		<< "\n"
		<< "Runs a deterministic fixed-step RK4 simulation with constant control.\n"
		<< "Convention: a_cmd is thrust acceleration magnitude opposing gravity (Up, -Down).\n"
		<< "Dynamics: d_dot = vd, vd_dot = g - a_cmd.\n"
		<< "\n"
		<< "Usage:\n"
		<< "  platform_runner [options]\n"
		<< "\n"
		<< "Options:\n"
		<< "  --to <float>				Start time (s). Default 0\n"
		<< "  --dt <float>				Time step (s). Default 0.01\n"
		<< "  --steps <int>				Number of steps. Default 200\n"
		<< "  --include-initial <0|1>	Include initial sample. Default 1\n"
		<< "\n"
		<< "  --d0 <float>				Initial down position (m). Default 0\n"
		<< "  --vd0 <float>				Initial down velocity (m/s). Default 0\n"
		<< "  --g <float>				Gravity magnitude (m/s^2). Dfault 9.80665\n"
		<< "  --a-cmd <float>			Thrust accel magnitude opposing gravity (m/s^2). Default 0\n"
		<< "\n"
		<< "  --output <path|stdout>	Output CSV path or 'stdout'. Default stdout\n"
		<< "							Output hygiene: file output requires a parent directory. \n"
		<< "  --help					Print this help\n";
}

bool is_stdout_token(const std::string& s) {
	return s.empty() || s == "stdout" || s == "-";
}

bool parse_bool_01(const std::string& s, bool& out) {
	if (s == "0") { out = false; return true; }
	if (s == "1") { out = true; return true; }
	return false;
}

int parse_args(const int argc, char** argv, AppConfig& cfg) {
	std::vector<std::string> args;
	args.reserve(static_cast<size_t>(argc));
	for (int i = 0; i < argc; ++i) args.emplace_back(argv[i]);

	for (int i = 1; i < argc; ++i) {
		const std::string& a = args[static_cast<size_t>(i)];

		if (a == "--help" || a == "-h") {
			print_help();
			return 2;
		}

		auto need_value = [&](const char* flag) -> const std::string* {
			if (i + 1 >= argc) return nullptr;
			const std::string& v = args[static_cast<size_t>(i+1)];
			if (!v.empty() && v[0] == '-') return nullptr;
			++i;
			return &args[static_cast<size_t>(i)];
		};

		if (a == "--t0") {
			const auto* v = need_value("--t0");
			if (!v) return fail("missing value for --t0");
			cfg.t0 = std::stod(*v);
		} else if (a == "--dt") {
			const auto* v = need_value("--dt");
			if (!v) return fail("missing value for --dt");
			cfg.dt = std::stod(*v);
		} else if (a == "--steps") {
			const auto* v = need_value("--steps");
			if (!v) return fail("missing value for --steps");
			cfg.steps = std::stoi(*v);
		} else if (a == "--include-initial") {
			const auto* v = need_value("--include-initial");
			if (!v) return fail("missing value for --include-initial");
			if (!parse_bool_01(*v, cfg.include_initial)) return fail("--include-initial must be 0 or 1");
		} else if (a == "--d0") {
			const auto* v = need_value("--d0");
			if (!v) return fail("missing value for --d0");
			cfg.d0 = std::stod(*v);
		} else if (a == "--vd0") {
			const auto* v = need_value("--vd0");
			if (!v) return fail("missing value for --vd0");
			cfg.vd0 = std::stod(*v);
		} else if (a == "--g") {
			const auto* v = need_value("--g");
			if (!v) return fail("missing value for --g");
			cfg.g = std::stod(*v);
		} else if (a == "--a-cmd") {
			const auto* v = need_value("--a-cmd");
			if (!v) return fail("missing value for --a-cmd");
			cfg.a_cmd = std::stod(*v);
		} else if (a == "--output") {
			const auto* v = need_value("--output");
			if (!v) return fail("missing value for --output");
			cfg.output = *v;
		} else {
			return fail("unknown argument: " + a);
		}
	}

	if (!(cfg.dt > 0.0)) return fail("dt must be > 0");
	if (cfg.steps <= 0) return fail("steps must be > 0");
	if (!(cfg.g > 0.0)) return fail("g must be < 0");

	return 0;
}

int write_trace_csv(std::ostream& os, const aerosyssim::sim::SimTracePlatform1D& tr) {
	os.imbue(std::locale::classic());
	os << std::scientific << std::setprecision(17);
	os << "t,d,vd,a_cmd\n";
	const size_t n = tr.t.size();
	for (size_t i = 0; i < n; ++i) {
		os << tr.t[i] << "," << tr.x[i][0] << "," << tr.x[i][1] << "," << tr.a_cmd[i] << "\n";
	}
	return 0;
}

int run(const AppConfig& cfg) {
	const auto x0 = aerosyssim::sim::make_platform1d_state(cfg.d0, cfg.vd0);
	const auto u = aerosyssim::sim::make_platform1d_control(cfg.a_cmd);
	auto p = aerosyssim::sim::make_platform1d_params_default();
	p.g = cfg.g;

	const auto tr = aerosyssim::sim::run_platform1d_fixed_step_constant_control(
		cfg.t0, cfg.dt, cfg.steps, cfg.include_initial, x0, u, p
	);

	if (is_stdout_token(cfg.output)) {
		return write_trace_csv(std::cout, tr);
	}

	const std::filesystem::path outp(cfg.output);
	const auto parent = outp.parent_path();
	if (parent.empty()) {
		return fail("output path must include a a parent directory (e.g., artifacts/run/trace.csv) or use stdout");
	}
	std::error_code ec;
	std::filesystem::create_directories(parent, ec);
	if (ec) {
		return fail("failed to create output directory: " + parent.string());
	}

	std::ofstream ofs(cfg.output);
	if (!ofs) {
		return fail("failed to open output fail: " + cfg.output);
	}
	return write_trace_csv(ofs, tr);
}

} // namespace

int main(int argc, char** argv) {
	AppConfig cfg{};
	const int rc = parse_args(argc, argv, cfg);
	if (rc == 2) return 0;
	if (rc != 0) return rc;
	return run (cfg);
}

