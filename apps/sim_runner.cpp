#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <string>
#include <locale>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>

#include "aerosyssim/sim/sim_engine.hpp"
#include "aerosyssim/sim/control.hpp"
#include "aerosyssim/sim/metrics.hpp"

namespace {

struct AppConfig {
	double t0 = 0.0;
	double dt = 0.01;
	std::size_t steps = 50;
	aerosyssim::math::Vec3 w0{0.3, -0.2, 0.1};
	std::string scenario = "principal_axis";
	bool w_user_set = false;

	bool torque_user_set = false;
	aerosyssim::math::Vec3 torque0{0.0, 0.0, 0.0};

	bool torque_step_user_set = false;
	double torque_step_t = 0.0;
	aerosyssim::math::Vec3 torque_step_0{0.0, 0.0, 0.0};
	aerosyssim::math::Vec3 torque_step_1{0.0, 0.0, 0.0};

	aerosyssim::math::Vec3 inertia_diag{2.0, 3.0, 4.0};
	bool inertia_full_user_set = false;
	aerosyssim::math::Mat3 inertia_full{
		2.0, 0.0, 0.0,
		0.0, 3.0, 0.0,
		0.0, 0.0, 4.0
	};

	std::string output_path;
	bool output_to_file = false;

	bool batch_mode = false;
	std::string batch_dir;
	std::string batch_outdir;
};

void print_help() {
	std::cout.imbue(std::locale::classic());
	std::cout 
		<< "sim_runner options: \n"
		<< " --dt <dt>\n"
		<< " --steps <N>\n"
		<< " --t0 <t0>\n"
		<< " --w <wx> <wy> <wz>\n"
		<< " --torque <tx> <ty> <tz>\n"
		<< " --torque-step <t_break> <tx0> <ty0> <tz0> <tx1> <ty1> <tz1>\n"
		<< " --scenario <principal_axis|coupled_rates>\n"
		<< " --config <path>\n"
		<< " --output <path>\n"
		<< " --batch <dir>\n"
		<< " --batch-outdir <dir>\n"
		<< "\n"
		<< "Batch mode: \n"
		<< " Reads all *.cfg files in <dir> and writes:\n"
		<< "  <batch-outdir>/summary.csv\n"
		<< "  <batch-outdir/<case>/trace.csv\n"
		<< "\n"
		<< "Config keys (key=value): dt, t0, steps, scenario, w, torque, torque-step, output, inertia_diag, inertia\n"
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

static std::string trim_copy(const std::string& s) {
	std::size_t a = 0;
	while (a < s.size() && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) {
		++a;
	}
	std::size_t b = s.size();
	while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) {
		--b;
	}
	return s.substr(a, b - a);

}

static bool is_regular_file(const std::string& path) {
	struct stat st;
	if (stat(path.c_str(), &st) != 0) {
		return false;
	}
	return S_ISREG(st.st_mode);
}

static bool ends_with(const std::string&s, const std::string& suffix) {
	if (s.size() < suffix.size()) return false;
	return s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static std::string basename_no_ext(const std::string& path) {
	std::string name = path;
	const std::size_t slash = name.find_last_of("/\\");
	if (slash != std::string::npos) {
		name = name.substr(slash + 1);
	}
	if (ends_with(name, ".cfg")) {
		name = name.substr(0, name.size() - 4);
	}
	return name;
}

static std::vector<std::string> list_cfg_files(const std::string& dir) {
	std::vector<std::string> out;
	DIR* d = opendir(dir.c_str());
	if (!d) {
		return out;
	}
	while (true) {
		errno = 0;
		dirent* ent = readdir(d);
		if (!ent) break;
		const std::string n = ent->d_name;
		if (n == "." || n == "..") continue;
		if (!ends_with(n, ".cfg")) continue;
		const std::string full = dir + "/" + n;
		if (is_regular_file(full)) {
			out.push_back(full);
		}
	}
	closedir(d);
	std::sort(out.begin(), out.end());
	return out;
}

static bool mkdir_p_one(const std::string& dir) {
	// Only one level (no recursion).
	if (dir.empty()) return false;
	if (::mkdir(dir.c_str(), 0777) == 0) return true;
	if (errno == EEXIST) return true;
	return false;
}

static bool parse_vec3(const std::string& value, aerosyssim::math::Vec3& out) {
	std::string v = value;
	for (char& c : v) {
		if (c == ',') {
			c = ' ';
		}
	}
	std::istringstream iss(v);
	double x = 0.0, y = 0.0, z = 0.0;
	if (!(iss >> x >> y >> z)) {
		return false;
	}
	std::string extra;
	if (iss >> extra) {
		return false;
	}
	out = aerosyssim::math::Vec3{x, y, z};
	return true;
}

static bool parse_torque_step7(const std::string& value,
		double& t_break,
		aerosyssim::math::Vec3& tau0,
		aerosyssim::math::Vec3& tau1) {
	std::string v = value;
	for (char& c : v) {
		if (c == ',') {
			c = ' ';
		}
	}
	std::istringstream iss(v);
	double tb = 0.0;
	double a0 = 0.0, a1 = 0.0, a2 = 0.0;
	double b0 = 0.0, b1 = 0.0, b2 = 0.0;
	if (!(iss >> tb >> a0 >> a1 >> a2 >> b0 >> b1 >> b2)) {
		return false;
	}
	std::string extra;
	if (iss >> extra) return false;
	
	t_break = tb;
	tau0 = aerosyssim::math::Vec3{a0, a1, a2};
	tau1 = aerosyssim::math::Vec3{b0, b1, b2};
	return true;
}

static bool parse_mat3_9(const std::string& value, aerosyssim::math::Mat3& out) {
	std::string v = value;
	for (char& c : v) if (c == ',') c = ' ';
	std::istringstream iss(v);
	aerosyssim::math::Mat3 A{};
	for (int i = 0; i < 9; ++i) {
		if (!(iss >> A[static_cast<std::size_t>(i)])) return false;
		}
	std::string extra;
	if (iss >> extra) return false;
	out = A;
	return true;
}

enum class ParseStatus {Ok, Help, Error};

static ParseStatus parse_config_file(const std::string& path, AppConfig& cfg) {
	std::ifstream ifs(path);
	if (!ifs) {
		std::cerr << "sim_runner: failed to open config file: " << path << "\n";
		return ParseStatus::Error;
	}

	std::string line;
	std::size_t lineno = 0;
	while (std::getline(ifs, line)) {
		++lineno;

		// Strip comments starting with '#'
		const std::size_t hash_pos = line.find('#');
		if (hash_pos != std::string::npos) {
			line = line.substr(0, hash_pos);
		}

		line = trim_copy(line);
		if (line.empty()) {
			continue;
		}

		const std::size_t eq = line.find('=');
		if (eq == std::string::npos) {
			std::cerr << "sim_runner: config parse error at " << path << ":" << lineno
					  << " (missing '=')\n";
			return ParseStatus::Error;
		}

		const std::string key = trim_copy(line.substr(0, eq));
		const std::string val = trim_copy(line.substr(eq + 1));

		if (key == "dt") {
			double tmp = 0.0;
			if (!parse_double(val, tmp)) {
				std::cerr << "sim_runner: config invalid dt at " << path << ":" << lineno << "\n";
				return ParseStatus::Error;
			}
			cfg.dt = tmp;
		} else if (key == "t0") {
			double tmp = 0.0;
			if (!parse_double(val, tmp)) {
				std::cerr << "sim_runner: config invalid t0 at " << path << ":" << lineno << "\n";
				return ParseStatus::Error;
			}
			cfg.t0 = tmp;
		} else if (key == "steps") {
			std::size_t tmp = 0;
			if (!parse_size(val, tmp)) {
				std::cerr << "sim_runner: config invalid steps at " << path << ":" << lineno << "\n";
				return ParseStatus::Error;
			}
			cfg.steps = tmp;
		} else if (key == "scenario") {
			cfg.scenario = val;
		} else if (key == "w") {
			aerosyssim::math::Vec3 wtmp;
			if (!parse_vec3(val, wtmp)) {
				std::cerr << "sim_runner: config invalid w at " << path << ":" << lineno << "\n";
				return ParseStatus::Error;
			}
			cfg.w0 = wtmp;
			cfg.w_user_set = true;
		} else if (key == "torque") {
			aerosyssim::math::Vec3 ttmp;
			if (!parse_vec3(val, ttmp)) {
				std::cerr << "sim_runner: config invalid torque at " << path << ":" << lineno << "\n";
				return ParseStatus::Error;
			}
			cfg.torque0 = ttmp;
			cfg.torque_user_set = true;
		} else if (key == "torque-step") {
			double tb = 0.0;
			aerosyssim::math::Vec3 tau0, tau1;
			if (!parse_torque_step7(val, tb, tau0, tau1)) {
				std::cerr << "sim_runner: config invalid torque_step at " << path << ":" << lineno << "\n";
				return ParseStatus::Error;
			}
			cfg.torque_step_t = tb;
			cfg.torque_step_0 = tau0;
			cfg.torque_step_1 = tau1;
			cfg.torque_step_user_set = true;
		} else if (key == "output") {
			if (val.empty() || val == "stdout") {
				cfg.output_to_file = false;
				cfg.output_path.clear();
			} else {
				cfg.output_to_file = true;
				cfg.output_path = val;
			}
		} else if (key == "inertia_diag") {
			aerosyssim::math::Vec3 d;
			if (!parse_vec3(val, d)) {
				std::cerr << "sim_runner: config invalid inertia_diag at "
						  << path << ":" << lineno
						  << " (expected inertia_diag=Ixx,Iyy,Izz)\n";
				return ParseStatus::Error;
			}
			cfg.inertia_diag = d;
		} else if (key == "inertia") {
			aerosyssim::math::Mat3 A;
			if (!parse_mat3_9(val, A)) {
				std::cerr << "sim_runner: config invalid inertia at "
						  << path << ":" << lineno
						  << " (expected 9 numbers row-major: "
						  << "inertia=a11,a12,a13,a21,a22,a23,a31,a32,a33)\n";
				return ParseStatus::Error;
			}
			cfg.inertia_full = A;
			cfg.inertia_full_user_set = true;
		} else {
			std::cerr << "sim_runner: unknown config key '" << key << "' at "
					  << path << ":" << lineno << "\n";
			return ParseStatus::Error;
		}
	}

	return ParseStatus::Ok;
}

ParseStatus parse_args(int argc, char** argv, AppConfig& cfg) {
	std::vector<std::string> args;
	args.reserve(static_cast<std::size_t>(argc));
	for (int i = 1; i < argc; ++i) {
		args.emplace_back(argv[i]);
	}

	bool saw_any_single_run_opt = false;
	bool saw_any_batch_opt = false;

	for (std::size_t i = 0; i < args.size(); ++i) {
		const std::string& a = args[i];
		if (a == "--help") {
			print_help();
			return ParseStatus::Help;
		} else if (a == "--dt") {
			saw_any_single_run_opt = true;
			if (i + 1 >= args.size() || !parse_double(args[i + 1], cfg.dt)) {
				std::cerr << "sim_runner: invalid --dt\n";
				return ParseStatus::Error;
			}
			i += 1;
		} else if (a == "--t0") {
			saw_any_single_run_opt = true;
			if (i + 1 >= args.size() || !parse_double(args[i + 1], cfg.t0)) {
				std::cerr << "sim_runner: invalid --t0\n";
				return ParseStatus::Error;
			}
			i += 1;
		} else if (a == "--steps") {
			saw_any_single_run_opt = true;
			if (i + 1 >= args.size() || !parse_size(args[i + 1], cfg.steps)) {
				std::cerr << "sim_runner: invalid --steps\n";
				return ParseStatus::Error;
			}
			i += 1;
		} else if (a == "--w") {
			saw_any_single_run_opt = true;
			double wx = 0.0, wy = 0.0, wz = 0.0;
			if (i + 3 >= args.size() || 
				!parse_double(args[i + 1], wx) ||
				!parse_double(args[i + 2], wy) ||
				!parse_double(args[i + 3], wz)) {
				std::cerr << "sim_runner: invalid --w\n";
				return ParseStatus::Error;
			}
			cfg.w0 = aerosyssim::math::Vec3{wx, wy, wz};
			cfg.w_user_set = true;
			i += 3;
		} else if (a == "--torque") {
			saw_any_single_run_opt = true;
			double tx = 0.0, ty = 0.0, tz = 0.0;
			if (i + 3 >= args.size() ||
				!parse_double(args[i + 1], tx) ||
				!parse_double(args[i + 2], ty) ||
				!parse_double(args[i + 3], tz)) {
				std::cerr << "sim_runner: invalid --torque\n";
				return ParseStatus::Error;
			}
			cfg.torque0 = aerosyssim::math::Vec3{tx, ty, tz};
			cfg.torque_user_set = true;
			i += 3;
		} else if (a == "--torque-step") {
			saw_any_single_run_opt = true;
			double tb = 0.0;
			double tx0 = 0.0, ty0 = 0.0, tz0 = 0.0;
			double tx1 = 0.0, ty1 = 0.0, tz1 = 0.0;
			if (i + 7 >= args.size() ||
				!parse_double(args[i + 1], tb) || 
				!parse_double(args[i + 2], tx0) || 
				!parse_double(args[i + 3], ty0) || 
				!parse_double(args[i + 4], tz0) || 
				!parse_double(args[i + 5], tx1) || 
				!parse_double(args[i + 6], ty1) || 
				!parse_double(args[i + 7], tz1)) { 
				std::cerr << "sim_runner: invalid -- torque-step\n";
				return ParseStatus::Error;
			}
			cfg.torque_step_t = tb;
			cfg.torque_step_0 = aerosyssim::math::Vec3{tx0, ty0, tz0};
			cfg.torque_step_1 = aerosyssim::math::Vec3{tx1, ty1, tz1};
			cfg.torque_step_user_set = true;
			i += 7;
		} else if (a == "--scenario") { 
			saw_any_single_run_opt = true;
			if (i + 1 >= args.size()) {
				std::cerr << "sim_runner: invalid --scenario\n";
				return ParseStatus::Error;
			}
			cfg.scenario = args[i + 1];
			i += 1;
		} else if (a == "--config") {
			saw_any_single_run_opt = true;
			if (i + 1 >= args.size()) {
				std::cerr << "sim_runner: invalid --config\n";
				return ParseStatus::Error;
			}
			// Config is handled in main via a pre-scan so that CLI always overrides config,
			// independent of argument ordering
			i += 1;
		} else if (a == "--output") {
			saw_any_single_run_opt = true;
			if (i + 1 >= args.size()) {
				std::cerr << "sim_runner: invalid --output\n";
				return ParseStatus::Error;
			}
			cfg.output_path = args[i + 1];
			cfg.output_to_file = true;
			i += 1;
		} else if (a == "--batch") {
			saw_any_batch_opt = true;
			if (i + 1 >= args.size()) {
				std::cerr << "sim_runner: invalid --batch\n";
				return ParseStatus::Error;
			}
			cfg.batch_mode = true;
			cfg.batch_dir = args[i + 1];
			i += 1;
		} else if (a == "--batch-outdir") {
			saw_any_batch_opt = true;
			if (i + 1 >= args.size()) {
				std::cerr << "sim_runner: invalid --batch-outdir\n";
				return ParseStatus::Error;
			}
			cfg.batch_outdir = args[i + 1];
			i += 1;
		} else {
			std::cerr << "sim_runner: unknown option: " << a << "\n";
			return ParseStatus::Error;
		}
	}

	if (saw_any_batch_opt && saw_any_single_run_opt) {
		std::cerr << "sim_runner: batch mode cannot be combined with single-run options\n";
		return ParseStatus::Error;
	}

	return ParseStatus::Ok;
}

} // namespace

static int run_batch(const AppConfig& batch_cfg) {
	if (batch_cfg.batch_dir.empty()) {
		std::cerr << "sim_runner: --batch requires a directory\n";
		return 1;
	}
	const std::string outdir = batch_cfg.batch_outdir.empty() ? std::string("artifacts/batch") : batch_cfg.batch_outdir;
	(void)mkdir_p_one("artifacts"); // ok if it fails. outdir may be elsewhere
	if (!mkdir_p_one(outdir)) {
		std::cerr << "sim_runner: failed to create batch outdir: " << outdir<< "\n";
		return 1;
	}

	const auto cfg_files = list_cfg_files(batch_cfg.batch_dir);
	if (cfg_files.empty()) {
		std::cerr << "sim_runner: no .cfg files found in : " << batch_cfg.batch_dir << "\n";
		return 1;
	}

	const std::string summary_path = outdir + "/summary.csv";
	std::ofstream summary(summary_path, std::ios::out | std::ios::trunc | std::ios::binary);
	if (!summary) {
		std::cerr << "sim_runner: failed to open summary file: " << summary_path << "\n";
		return 1;
	}

	summary.imbue(std::locale::classic());
	summary.setf(std::ios::scientific);
	summary << std::setprecision(17);
	summary
		<< "case,config_path,trace_path,samples,t_final,has_inertia_full,"
		<< "qnorm_max_abs_err,energy_rel_dirft,Lnorm_rel_drift,"
		<<"wx_final,wy_final,wz_final,w_norm_min,w_norm_max\n";

	for (const auto& cfg_path : cfg_files) {
		AppConfig cfg;
		const auto st_cfg = parse_config_file (cfg_path, cfg);
		if (st_cfg != ParseStatus::Ok) return 1;

		const std::string case_name = basename_no_ext(cfg_path);
		const std::string case_dir = outdir + "/" + case_name;
		if (!mkdir_p_one(case_dir)) {
			std::cerr << "sim_runner: failed to create case dir: " << case_dir << "\n";
			return 1;
		}

		// Overwrite output for hygiene in batch mode
		const std::string trace_path = case_dir + "/trace.csv";
		cfg.output_to_file = true;
		cfg.output_path = trace_path;

		// Scenario defualts (only if config did not explicitly provide w)
		if (cfg.scenario == "principal_axis") {
			if (!cfg.w_user_set) cfg.w0 = aerosyssim::math::Vec3{0.3, -0.2, 0.1};
		} else if (cfg.scenario == "coupled_rates") {
			if (!cfg.w_user_set) cfg.w0 = aerosyssim::math::Vec3{0.6, 0.4, -0.3};
		} else {
			std::cerr << "sim_runner: unknown scenario: " << cfg.scenario << " in " << cfg_path << "\n";
			return 1;
		}

		using aerosyssim::sim::AttitudeState;
		using aerosyssim::sim::AttitudeControl;
		using aerosyssim::sim::RigidBodyParams;
		using aerosyssim::sim::SimConfigFixedStep;

		AttitudeState x0;
		x0.q_wxyz = aerosyssim::math::Quat{1.0, 0.0, 0.0, 0.0};
		x0.w_body = cfg.w0;

		RigidBodyParams p{cfg.inertia_diag};
		if (cfg.inertia_full_user_set) {
			p.inertia_body = cfg.inertia_full;
		}

		SimConfigFixedStep scfg;
		scfg.t0 = cfg.t0;
		scfg.dt = cfg.dt;
		scfg.num_steps = cfg.steps;
		scfg.include_initial = true;

		const aerosyssim::sim::AttitudeControlFn u_of_t_x = [&](double t, const AttitudeState&) {
			if (cfg.torque_step_user_set) {
				const auto tau = (t < cfg.torque_step_t) ? cfg.torque_step_0 : cfg.torque_step_1;
				return AttitudeControl{tau};
			}
			if (cfg.torque_user_set) return AttitudeControl{cfg.torque0};
			return AttitudeControl{aerosyssim::math::Vec3{0.0, 0.0, 0.0}};
		};

		const auto tr = aerosyssim::sim::run_attitude_fixed_step(scfg, x0, u_of_t_x, p);

		// Write trace.csv
		std::ofstream ofs(trace_path, std::ios::out | std::ios::trunc | std::ios::binary);
		if (!ofs) {
			std::cerr << "sim_runner: failed to open trace file: " << trace_path << "\n";
			return 1;
		}
		ofs.imbue(std::locale::classic());
		ofs.setf(std::ios::scientific);
		ofs << std::setprecision(17);
		ofs << "t,qw,qx,qy,qz,wx,wy,wz\n";
		for (std::size_t i = 0; i < tr.x.size(); ++i) {
			const auto& xi = tr.x[i];
			ofs << tr.t[i] << ","
						   << xi[0] << "," << xi[1] << "," << xi[2] << "," << xi[3] << ","
						   << xi[4] << "," << xi[5] << "," << xi[6] << "\n";
		}

		// Metrics
		const auto stats = aerosyssim::sim::compute_trace_invariants(tr, p);
		double w_norm_min = std::numeric_limits<double>::infinity();
		double w_norm_max = -std::numeric_limits<double>::infinity();
		for (const auto& xi : tr.x) {
			const double wx = xi[4], wy = xi[5], wz = xi[6];
			const double wn = std::sqrt(wx*wx + wy*wy + wz*wz);
			w_norm_min = std::min(w_norm_min, wn);
			w_norm_max = std::max(w_norm_max, wn);
		}
		const auto& xf = tr.x.back();

		summary
			<< case_name << ","
			<< cfg_path << ","
			<< trace_path << ","
			<< tr.x.size() << ","
			<< tr.t.back() << ","
			<< (cfg.inertia_full_user_set ? 1 : 0) << ","
			<< stats.qnorm_max_abs_err << ","
			<< stats.energy_rel_drift << ","
			<< stats.Lnorm_rel_drift << ","
			<< xf[4] << "," << xf[5] << "," << xf[6] << ","
			<< w_norm_min << "," << w_norm_max 
			<< "\n";
	}

	std::cout << "Batch complete.\n";
	std::cout << " outdir: " << outdir << "\n";
	std::cout << " summary: " << outdir << "/summary.csv\n";
	return 0;
}

int main(int argc, char** argv) {
	using aerosyssim::sim::AttitudeState;
	using aerosyssim::sim::AttitudeControl;
	using aerosyssim::sim::RigidBodyParams;
	using aerosyssim::sim::SimConfigFixedStep;

	AppConfig app_cfg;

	// Pre-scan for --config so config is applied before CLI parsing
	// This guarantees CLI options override config regardless of ordering
	std::string config_path;
	for (int i = 1; i < argc; ++i) {
		if (std::string(argv[i]) == "--config") {
			if (i + 1 >= argc) {
				std::cerr << "sim_runner: invalid --config\n";
				return 1;
			}
			config_path = argv[i + 1];
			++i;
		}
	}

	if (!config_path.empty()) {
		const auto st_cfg = parse_config_file(config_path, app_cfg);
		if (st_cfg != ParseStatus::Ok) {
			return 1;
		}
	}

	const auto st = parse_args(argc, argv, app_cfg);
	if (st == ParseStatus::Help) {
		return 0;
	}
	if (st == ParseStatus::Error) {
		return 1;
	}

	// TESTING
	if (app_cfg.batch_mode) return run_batch(app_cfg);

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


	RigidBodyParams p{app_cfg.inertia_diag};
	if (app_cfg.inertia_full_user_set) {
		p.inertia_body = app_cfg.inertia_full;
	}
	SimConfigFixedStep cfg;
	cfg.t0 = app_cfg.t0;
	cfg.dt = app_cfg.dt;
	cfg.num_steps = app_cfg.steps;
	cfg.include_initial = true;

	// Control selection: piecewise schedule > constant torque > default 0 torque
	const aerosyssim::sim::AttitudeControlFn u_of_t_x = [&](double t, const AttitudeState&) {
		if (app_cfg.torque_step_user_set) {
			const auto tau = (t < app_cfg.torque_step_t) ? app_cfg.torque_step_0 : app_cfg.torque_step_1;
			return AttitudeControl{tau};
		}
		if (app_cfg.torque_user_set) {
			return AttitudeControl{app_cfg.torque0};
		}
		return AttitudeControl{aerosyssim::math::Vec3{0.0, 0.0, 0.0}};
	};

	const auto trace = aerosyssim::sim::run_attitude_fixed_step(cfg, x0, u_of_t_x, p);

	std::ofstream ofs;
	std::ostream* osp = &std::cout;
	
	if (app_cfg.output_to_file) {
		if (app_cfg.output_path.empty()) {
			std::cerr << "sim_runner: invalid --output\n";
			return 1;
		}
		ofs.open(app_cfg.output_path, std::ios::out | std::ios::trunc | std::ios::binary);
		if (!ofs) {
			std::cerr << "sim_runner: failed to open output file: " << app_cfg.output_path << "\n";
			return 1;
		}
		osp = &ofs;
	}
	std::ostream& os = *osp;


	// Stable CSV formatting (applied to selected output stream)
	os.imbue(std::locale::classic());
	os.setf(std::ios::scientific);
	os << std::setprecision(17);

	os << "t,qw,qx,qy,qz,wx,wy,wz\n";
	for (std::size_t i = 0; i < trace.x.size(); ++i) {
		const auto& xi = trace.x[i];
		os << trace.t[i] << ","
		   << xi[0] << "," << xi[1] << "," << xi[2] << "," << xi[3] << ","
		   << xi[4] << "," << xi[5] << "," << xi[6] << "\n";
	}

	return 0;

}
