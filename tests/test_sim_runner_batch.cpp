#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_runner_batch: FAIL: " << msg << "\n";
	return 1;
}

bool file_exists_nonempty(const std::string& path) {
	struct stat st;
	if (stat(path.c_str(), &st) != 0) return false;
	return S_ISREG(st.st_mode) && st.st_size > 0;
}

int mkdir_if_needed(const std::string& p) {
	if (::mkdir(p.c_str(), 0777) == 0) return 0;
	return (errno == EEXIST) ? 0 : 1;
}

int run_cmd(const std::string& cmd) {
	const int rc = std::system(cmd.c_str());
		if (rc == -1) return 1;
	return (rc == 0) ? 0 : 1;
}

std::vector<std::string> split_lines(const std::string& s) {
	std::vector<std::string> lines;
	std::string cur;
	for (char c : s) {
		if (c == '\n') {
			lines.push_back(cur);
			cur.clear();
		} else if (c != '\r') {
			cur.push_back(c);
		}
	}
	if (!cur.empty()) lines.push_back(cur);
	return lines;
}

std::string slurp(const std::string& path) {
	std::ifstream ifs(path, std::ios::in | std::ios::binary);
	if (!ifs) return {};
	std::string s((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
	return s;
}

} // namespace 

int main() {
	const std::string cfg_dir = "tmp_batch_cfgs";
	const std::string out_dir = "tmp_batch_out";

	if (mkdir_if_needed(cfg_dir) != 0) return fail("failed to create tmp_batch_cfgs");
	if (mkdir_if_needed(out_dir) != 0) return fail("failed to create tmp_batch_out");

	// Case A: diagonal inertia
	{
		std::ofstream f(cfg_dir + "/case_diag.cfg", std::ios::out | std::ios::trunc);
		if (!f) return fail("failed to write case_diag.cfg");
		f
			<< "dt=1e-3\n"
			<< "t0=0.0\n"
			<< "steps=10\n"
			<< "scenario=principal_axis\n"
			<< "w=0.3,-0.2,0.1\n"
			<< "torque=0,0,0\n"
			<< "inertia_diag=2,3,4\n";
	}

	// Case B: full inertia enabled
	{
		std::ofstream f(cfg_dir + "/case_full.cfg", std::ios::out | std::ios::trunc);
		if (!f) return fail("failed to write case_full.cfg");
		f
			<< "dt=1e-3\n"
			<< "t0=0.0\n"
			<< "steps=10\n"
			<< "scenario=principal_axis\n"
			<< "w=1,0,0\n"
			<< "torque=0,0,0\n"
			<< "inertia_diag=2,2,3\n"
			<< "inertia=2,0.5,0,0.5,2,0,0,0,3\n";
	}

	const std::string cmd = std::string("../bin/sim_runner --batch ") + cfg_dir + " --batch-outdir " + out_dir;
	if (run_cmd(cmd) != 0) return fail("sim_runner batch command failed");

	const std::string summary_path = out_dir + "/summary.csv";
	if (!file_exists_nonempty(summary_path)) return fail("summary.csv missing or empty");

	const std::string summary = slurp(summary_path);
	const auto lines = split_lines(summary);
	if (lines.size() != 3) return fail("expected header + 2 rows in summary.csv");
	if (lines[0].find("case,config_path,trace_path") != 0) return fail("summary header mismatch");

	// Ensure per-case traces exist
	if (!file_exists_nonempty(out_dir + "/case_diag/trace.csv")) return fail("case_diag trace missing");
	if (!file_exists_nonempty(out_dir + "/case_full/trace.csv")) return fail("case_full trace missing");

	// Cleanup: remove files created (leave dirs if rmdir fails)
	std::remove((cfg_dir + "/case_diag.cfg").c_str());
	std::remove((cfg_dir + "/case_full.cfg").c_str());
	std::remove((out_dir + "/summary.csv").c_str());
	std::remove((out_dir + "/case_diag/trace.csv").c_str());
	std::remove((out_dir + "/case_full/trace.csv").c_str());
	::rmdir((out_dir + "/case_diag").c_str());
	::rmdir((out_dir + "/case_full").c_str());
	::rmdir(out_dir.c_str());
	::rmdir(cfg_dir.c_str());

	return 0;
}
