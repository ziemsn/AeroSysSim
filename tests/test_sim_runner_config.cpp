#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sys/wait.h>

namespace {

int fail(const char* msg) {
	std::cerr << "times_sim_runner_config: FAIL: " << msg << "\n";
	return 1;
}

struct CmdResult {
	int exit_code = -1;
	std::string out;
};

CmdResult run_and_capture(const char* cmd) {
	CmdResult r;
	FILE* pipe = popen(cmd, "r");
	if (!pipe) {
		return r;
	}
	char buf[4096];
	while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
		r.out += buf;
	}
	const int status = pclose(pipe);
	if (status != -1 && WIFEXITED(status)) {
		r.exit_code = WEXITSTATUS(status);
	}
	return r;
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
	if (!cur.empty()) {
		lines.push_back(cur);
	}
	return lines;
}

} // namespace

int main() {
	// Ctest runs from build/ so we can create the config in the current dir.
	const std::string cfg_path = "sim_runner_test.cfg";

	{
		std::ofstream ofs(cfg_path, std::ios::out | std::ios::trunc);
		if (!ofs) {
			return fail("failed to create config file in build directory");
		}
		ofs << "# sim_runner config test\n";
		ofs << "dt=0.01\n";
		ofs << "t0=0.0\n";
		ofs << "steps=10\n";
		ofs << "scenario=coupled_rates\n";
		ofs << "torque-step=0.05,1,0,0,0,2,0\n";
		// Leave output unset so CSV goes to stdout for this test.
	}

	// CLI should override config regardless of ordering 
	// Config says steps=10, CLI says steps=11.
	const char* cmd = "LC_ALL=C ../bin/sim_runner --steps 11 --config sim_runner_test.cfg";

	const auto ra = run_and_capture(cmd);
	const auto rb = run_and_capture(cmd);

	if (ra.exit_code != 0 || rb.exit_code != 0) {
		return fail("sim_runner exited nonzer when driven by config");
	}
	if (ra.out.empty() || rb.out.empty()) {
		return fail("sim_runner output was empty");
	}
	if (ra.out != rb.out) {
		return fail("output not byte identical across identical config-driven runs");
	}

	const auto lines = split_lines(ra.out);
	if (lines.empty()) {
		return fail("no CSV lines produced");
	}
	if (lines[0] != "t,qw,qx,qy,qz,wx,wy,wz") {
		return fail("CSV header mismatch");
	}

	// include_initial=ture and steps=11 -> 1 header + 12 data lines = 13
	if (lines.size() != 13) {
		return fail("unexpected number of CSV lines for steps-11 via config + CLI override");
	}

	std::remove(cfg_path.c_str());
	return 0;
}


























