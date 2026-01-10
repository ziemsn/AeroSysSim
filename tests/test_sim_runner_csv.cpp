#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_runner_csv: FAIL: " << msg << "\n";
	return 1;
}

std::string run_and_capture(const char* cmd) {
	std::string out;
	FILE* pipe = popen(cmd, "r");
	if (!pipe) {
		return out;
	}

	char buf[4096];
	while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
		out += buf;
	}
	pclose(pipe);
	return out;
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
	const char* cmd = "../bin/sim_runner";

	const std::string a = run_and_capture(cmd);
	const std::string b = run_and_capture(cmd);

	if (a.empty() || b.empty()) {
		return fail("sim_runner output was empty");
	}

	if (a != b) {
		return fail("sim_runner output not byte identical across identical runs");
	}

	const auto lines = split_lines(a);
	if (lines.size() < 2) {
		return fail("sim_runner output should include header an;d at least one data line");
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

	return 0;
}


