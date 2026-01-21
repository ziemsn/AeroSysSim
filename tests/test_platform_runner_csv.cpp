#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/wait.h>
#include <vector>

namespace {

int fail(const char* msg) {
	std::cerr << "test_platform_runner_csv: FAIL: " << msg << "\n";
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

	const auto h = run_and_capture("../bin/platform_runner --help");
	if (h.exit_code != 0) return fail("--help should exit with code 0");
	if (h.out.find("platform_runner") == std::string::npos) {
		return fail("--help output did not contain expected usage text");
	}
	
	auto check_csv = [&](const char* cmd) -> int {
		const auto ra = run_and_capture(cmd);
		const auto rb = run_and_capture(cmd);
		if (ra.exit_code != 0 || rb.exit_code != 0) {
			return fail("platform_runner exited nonzero");
		}

		const std::string& a = ra.out;
		const std::string& b = rb.out;
		if (a.empty() || b.empty()) return fail("platform_runner output was empty");
		if (a != b) return fail("platform_runner ouptut was not byte identical across runs");

		const auto lines = split_lines(a);
		if (lines.size() < 2) {
			return fail("platform_runner output should include header and at least one data line");
		}

		if (lines[0] != "t,d,vd,a_cmd") return fail("CSV header mismatch");

		// Basic structural check: first data line should have 4 comma-seperated fields
		int commas = 0;
		for (char c : lines[1]) {
			if (c == ',') ++commas;
		}
		if (commas != 3) return fail("first data line does not have 4 fields");

		return 0;
	};

	// include_initial=1 and steps=10 => 1 header + 11 data lines = 12
	if (check_csv("LC_ALL=C ../bin/platform_runner --dt 0.01 --steps 10 --include-initial 1 --t0 0.0 --d0 0.0 --vd0 1.25 --g 9.80665 --a-cmd 0.0") != 0) {
		return 1;
	}

	// Hover case: a_cmd = g (still deterministic)
	if (check_csv("LC_ALL=C ../bin/platform_runner --dt 0.01 --steps 10 --include-initial 1 --t0 0.0 --d0 05.0 --vd0 1.25 --g 9.80665 --a-cmd 9.80665") != 0) {
		return 1;
	}
		

	// Inluclude_initial=0 and steps=10 => 1 hdeader + 10 data lines = 11
	{
		const auto r = run_and_capture("LC_ALL=C ../bin/platform_runner --dt 0.01 --steps 10 --include-initial 0 --t0 0.0 --d0 0.0 --vd0 0.0 --g 9.80665 --a-cmd 0.0");
		if (r.exit_code != 0) return fail("platform_runner exited nonzero");
		const auto lines = split_lines(r.out);
		if (lines.size() != 11) return fail("unexpected number of CSV lines for include_initial=0, steps=10");
	}

	return 0;
}


