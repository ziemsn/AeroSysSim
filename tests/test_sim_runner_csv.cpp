#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <sys/wait.h>

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_runner_csv: FAIL: " << msg << "\n";
	return 1;
}

struct CmdResult {
	std::string out;
	int exit_code = -1;
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
	} else {
		r.exit_code = -1;
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
	// Ctest runs from build/ so ../bin/sim_runner should be valid.
	const auto h = run_and_capture("../bin/sim_runner --help");
	if (h.exit_code != 0) {
		return fail("--help should exit with code 0");
	}
	if (h.out.find("sim_runner options") == std::string::npos) {
		return fail("--help output did not contain expected usage text");
	}
	
	auto check_csv = [&](const char* cmd) -> int {
		const auto ra = run_and_capture(cmd);
		const auto rb = run_and_capture(cmd);
		if (ra.exit_code != 0 || rb.exit_code != 0) {
			return fail("sim_runner exited nonzero");
		}

		const std::string& a = ra.out;
		const std::string& b = rb.out;
		if (a.empty() || b.empty()) {
			return fail("sim_runner output was empty");
		}

		if (a != b) {
			return fail("sim_runner output not byte identical across identical runs");
		}

		const auto lines = split_lines(a);
		if (lines.size() < 2) {
			return fail("sim_runner output should include header and at least one data line");
		}

		if (lines[0] != "t,qw,qx,qy,qz,wx,wy,wz") {
			return fail("CSV header mismatch");
		}

		// Basic structural check: first data line should have 8 comma-separated fields
		int commas = 0;
		for (char c: lines[1]) {
			if (c == ','){
				++commas;
			}
		}
		if (commas != 7) {
			return fail("first data line does not have 8 fields");
		}
		
		// With include_initial=true and steps=10:
		// total lines = 1 header + 11 data lines = 12
		if (lines.size() != 12) {
			return fail("unexpected number of CSV lines for steps=10");
		}
		return 0;
	};

	// Scenario 1: explicit w (principal axis style)
	if (check_csv("LC_ALL=C ../bin/sim_runner --dt 0.01 --steps 10 --w 0.3 -0.2 0.1 --t0 0.0") != 0) {
		return 1;
	}

	// Scenario 2: coupled rates preset (do not pass --w; scenario should supply defaults)
	if (check_csv("LC_ALL=C ../bin/sim_runner --scenario coupled_rates --dt 0.01 --steps 10 --t0 0.0") != 0) {
		return 1;
	}
	// Scenario 3: User specified --torque-step
	if (check_csv("LC_ALL=C ../bin/sim_runner --dt 0.01 --steps 10 --torque-step 0.05 1 0 0 0 2 0") != 0) {
		return 1;
	}

	return 0;
}


