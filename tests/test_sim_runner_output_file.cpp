#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/wait.h>

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_runner_output_file: FAIL: " << msg << "\n";
	return 1;
}	

int run_cmd(const char* cmd) {
	const int status = std::system(cmd);
	if (status == -1) {
		return -1;
	}
	if (WIFEXITED(status)) {
		return WEXITSTATUS(status);
	}
	return -1;
}

std::string read_file(const std::string& path) {
	std::ifstream ifs(path, std::ios::in | std::ios::binary);
	if (!ifs) {
		return {};
	}
	std::string s((std::istreambuf_iterator<char>(ifs)),
				   std::istreambuf_iterator<char>());
	return s;
}

} // namespace

int main() {
	const std::string out_path = "test_sim_runner_out.csv";
	std::remove(out_path.c_str());

	const char* cmd1 = 
		"LC_ALL=C ../bin/sim_runner --dt 0.01 --steps 10 --t0 0.0 "
		"--torque-step 0.05 1 0 0 0 2 0 "
		"--output test_sim_runner_out.csv";

	const char* cmd2 = 
		"LC_ALL=C ../bin/sim_runner --dt 0.01 --steps 10 --t0 0.0 "
		"--torque-step 0.05 1 0 0 0 2 0 "
		"--output test_sim_runner_out.csv";
	
	if (run_cmd(cmd1) != 0) {
		return fail("first sim_runner output run exited nonzero");
	}
	const std::string a = read_file(out_path);
	if (a.empty()) {
		return fail("output file missing or empy after first run");
	}

	if (run_cmd(cmd2) != 0) {
		return fail("second sim_runner output run exited nonzero");
	}
	const std::string b = read_file(out_path);
	if (b.empty()) {
		return fail("output file missing or empy after seconde run");
	}

	if (a != b) {
		return fail("output file content not byte identical across identical runs");
	}

	// Basic structure checks
	const std::string header = "t,qw,qx,qy,qz,wx,wy,wz\n";
	if (a.rfind(header, 0) != 0) {
		return fail("CSV hader mismatch in output file");
	}

	// With include_initial=true and steps=10:
	// total lines = 1 header + 11 data lines = 12 lines
	int newlines = 0;
	for (char c: a) {
		if (c == '\n') {
			++newlines;
		}
	}
	if (newlines != 12) {
		return fail("unexpected number of CSV lines in output file for steps=10");
	}

	std::remove(out_path.c_str());
	return 0;
}

