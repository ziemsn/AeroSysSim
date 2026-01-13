#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

int fail(const char* msg) {
	std::cerr << "test_sim_runner_config_inertia:FAIL: " << msg << "\n";
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

std::vector<std::string> split_commas(const std::string& s) {
	std::vector<std::string> parts;
	std::string cur;
	for (char c : s) {
		if (c == ',') {
			parts.push_back(cur);
			cur.clear();
		} else {
			cur.push_back(c);
		}
	}
	parts.push_back(cur);
	return parts;
}

bool parse_double(const std::string& s, double& out) {
	char* end = nullptr;
	out = std::strtod(s.c_str(), &end);
	return end && *end == '\0';
}

} // namespace

int main() {
	// CTest runs from build/ so this fiule will be created in build/ and
	//  ../bin/sim_runner shuold exist
	const std::string cfg_path = "tmp_sim_runner_full_inertia.cfg";

	// Construct a case where diagonal inertia would yield wdot = 0 but full inertia yeilds wdot != 0.
	
	{
		std::ofstream ofs(cfg_path, std::ios::out | std::ios::trunc);
		if (!ofs) {
			return fail("failed to create temporary config file");
		}
		ofs
			<< "dt=1e-6\n"
			<< "t0=0.0\n"
			<< "steps=1\n"
			<< "w=1,0,0\n"
			<< "torque=0,0,0\n"
			<< "inertia_diag=2,2,3\n"
			<< "inertia=2,0.5,0,0.5,2,0,0,0,3\n"
			<< "output=stdout\n";
	}

	const std::string cmd = std::string("../bin/sim_runner --config ") + cfg_path;
	const std::string out = run_and_capture(cmd.c_str());

	std::remove(cfg_path.c_str());

	if (out.empty()) {
		return fail("sim_runner output was empty");
	}

	const auto lines = split_lines(out);
	if (lines.size() != 3) {
		return fail("expected 1 header + 2 datalines for steps=1 with include_initial = true");
	}
	if (lines[0] != "t,qw,qx,qy,qz,wx,wy,wz") {
		return fail("CSV header mismatch");
	}
	
	const auto fields = split_commas(lines[2]); // second data line (t=dt)
	if (fields.size() != 8) {
		return fail("expected 8 CSV fields on second data line");
	}

	double wz = 0.0;
	if (!parse_double(fields[7], wz)) {
		return fail("failed to parse wz field");
	}

	const double dt = 1.0e-6;
	const double expected_wz = (-1.0 / 6.0) * dt;
	const double tol = 1.0e-12;
	if (std::fabs(wz - expected_wz) > tol) {
		std::cerr << "got wz=" << wz << "expected wz=" << expected_wz << "\n";
		return fail("full inertia config did not produce expected wz update");
	}

	return 0;
}






















