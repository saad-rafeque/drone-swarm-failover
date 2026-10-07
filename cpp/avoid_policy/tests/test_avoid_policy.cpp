// Checks the C++ port against the Python reference, and that bad weight files are rejected.
// Usage: test_avoid_policy <avoid_policy.bin> <reference_cases.txt>
// Exit code 0 means every check passed. No test framework: plain asserts that print and count.
#include "avoid_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ap = avoid_policy;

namespace {

int g_failures = 0;

void Expect(bool ok, const std::string& what) {
  if (!ok) {
    ++g_failures;
    std::cout << "FAIL: " << what << "\n";
  }
}

// The observation is single precision. Python and C++ may round cos/sin one unit apart on another
// C library, so allow a few units in the last place rather than exact equality.
constexpr double kObsTol = 1e-6;
// The action sums 128 products per neuron in a different order than numpy does.
constexpr double kActTol = 1e-9;

struct Case {
  ap::AvoidInput in;
  ap::Observation obs{};
  ap::Action act{};
  ap::Vec2 corr;
};

std::vector<Case> ReadCases(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::vector<Case> cases;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream s(line);
    Case c;
    s >> c.in.heading >> c.in.v_des.x >> c.in.v_des.y >> c.in.vel.x >> c.in.vel.y >>
        c.in.slot_err.x >> c.in.slot_err.y;
    for (double& r : c.in.rays) s >> r;
    int has_nearest = 0;
    ap::Vec2 nearest;
    s >> has_nearest >> nearest.x >> nearest.y;
    if (has_nearest) c.in.nearest = nearest;
    std::size_t n = 0;
    s >> n;
    c.in.neighbors.resize(n);
    for (ap::Neighbor& nb : c.in.neighbors) s >> nb.rx >> nb.ry >> nb.rvx >> nb.rvy;
    for (float& o : c.obs) s >> o;
    s >> c.act[0] >> c.act[1] >> c.corr.x >> c.corr.y;
    if (!s) throw std::runtime_error("malformed reference case in " + path);
    cases.push_back(c);
  }
  return cases;
}

void TestAgainstPython(const ap::Policy& policy, const std::vector<Case>& cases) {
  double max_obs = 0.0, max_act = 0.0, max_corr = 0.0;
  for (const Case& c : cases) {
    const ap::Observation obs = ap::BuildObservation(c.in);
    for (std::size_t i = 0; i < ap::kObsDim; ++i) {
      max_obs = std::max(max_obs, std::fabs(static_cast<double>(obs[i]) - static_cast<double>(c.obs[i])));
    }
    // Feed the Python observation, so this isolates the network from the observation code.
    const ap::Action act = policy.Act(c.obs);
    for (std::size_t i = 0; i < ap::kActDim; ++i) max_act = std::max(max_act, std::fabs(act[i] - c.act[i]));
    // End to end: observation + network + rotation back to East-North.
    const ap::Vec2 corr = policy.Correction(c.in);
    max_corr = std::max({max_corr, std::fabs(corr.x - c.corr.x), std::fabs(corr.y - c.corr.y)});
  }
  std::printf("%zu reference cases: max |error|  observation %.3g  action %.3g  correction %.3g\n",
              cases.size(), max_obs, max_act, max_corr);
  Expect(!cases.empty(), "the reference file has cases");
  Expect(max_obs <= kObsTol, "observation matches Python");
  Expect(max_act <= kActTol, "action matches Python");
  Expect(max_corr <= 1e-4, "end-to-end correction matches Python");
}

void TestNeighbourSelection() {
  ap::AvoidInput in;  // heading 0: the heading frame is the East-North frame
  in.neighbors = {{30.0, 0.0, 0, 0},   // out of range: ignored
                  {9.0, 0.0, 0, 0},
                  {3.0, 0.0, 1, 0},
                  {6.0, 0.0, 0, 0},
                  {12.0, 0.0, 0, 0},   // fourth nearest: dropped
                  {3.0, 0.0, 2, 0}};   // same distance as the third entry: stays behind it
  const ap::Observation o = ap::BuildObservation(in);
  const std::size_t base = 8 + ap::kNumRays;
  Expect(std::fabs(o[base] - 3.0f / 25.0f) < 1e-7f && std::fabs(o[base + 2] - 0.1f) < 1e-7f, "nearest neighbour first");
  Expect(std::fabs(o[base + 5] - 3.0f / 25.0f) < 1e-7f && std::fabs(o[base + 7] - 0.2f) < 1e-7f, "ties keep input order");
  Expect(std::fabs(o[base + 10] - 6.0f / 25.0f) < 1e-7f, "third nearest neighbour");
  Expect(o[base + 4] == 1.0f && o[base + 9] == 1.0f && o[base + 14] == 1.0f, "three slots are marked filled");

  in.neighbors.clear();
  const ap::Observation empty = ap::BuildObservation(in);
  bool all_zero = true;
  for (std::size_t i = base; i < ap::kObsDim; ++i) all_zero = all_zero && empty[i] == 0.0f;
  Expect(all_zero, "no neighbours leaves the neighbour slots at zero");
}

void TestCorrectionIsClipped() {
  const ap::Vec2 c = ap::ActionToCorrection({5.0, -5.0}, 0.0);
  Expect(c.x == ap::kMaxCorrectionMps && c.y == -ap::kMaxCorrectionMps, "actions outside [-1, 1] are clipped");
}

bool LoadThrows(const std::string& path) {
  try {
    ap::Policy::Load(path);
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

void TestBadFilesAreRejected(const std::string& good_path) {
  std::ifstream f(good_path, std::ios::binary);
  const std::string good((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  const auto write = [](const std::string& path, const std::string& bytes) {
    std::ofstream(path, std::ios::binary).write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  };

  Expect(LoadThrows("no_such_file.bin"), "a missing file is rejected");

  std::string bad_magic = good;
  bad_magic[0] = 'X';
  write("bad_magic.bin", bad_magic);
  Expect(LoadThrows("bad_magic.bin"), "a wrong magic number is rejected");

  write("truncated.bin", good.substr(0, good.size() / 2));
  Expect(LoadThrows("truncated.bin"), "a truncated file is rejected");

  write("trailing.bin", good + "x");
  Expect(LoadThrows("trailing.bin"), "trailing bytes are rejected");

  std::string bad_shape = good;
  bad_shape[16] = static_cast<char>(bad_shape[16] + 1);  // first layer: cols 47 -> 48
  write("bad_shape.bin", bad_shape);
  Expect(LoadThrows("bad_shape.bin"), "a layer that does not fit the observation is rejected");

  for (const char* name : {"bad_magic.bin", "truncated.bin", "trailing.bin", "bad_shape.bin"}) std::remove(name);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: test_avoid_policy <avoid_policy.bin> <reference_cases.txt>\n";
    return 2;
  }
  try {
    const ap::Policy policy = ap::Policy::Load(argv[1]);
    Expect(policy.hidden_layer_count() == 2, "the shipped policy has two hidden layers");
    TestAgainstPython(policy, ReadCases(argv[2]));
    TestNeighbourSelection();
    TestCorrectionIsClipped();
    TestBadFilesAreRejected(argv[1]);
  } catch (const std::exception& e) {
    std::cout << "FAIL: unexpected exception: " << e.what() << "\n";
    return 1;
  }
  std::cout << (g_failures == 0 ? "all checks passed\n" : "some checks failed\n");
  return g_failures == 0 ? 0 : 1;
}
