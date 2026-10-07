// Times one avoidance step (observation + network + rotation) on this machine.
// Usage: bench_avoid_policy <avoid_policy.bin> [iterations]
#include "avoid_policy.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>

namespace ap = avoid_policy;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: bench_avoid_policy <avoid_policy.bin> [iterations]\n";
    return 2;
  }
  const long iterations = argc > 2 ? std::atol(argv[2]) : 200000;
  try {
    const ap::Policy policy = ap::Policy::Load(argv[1]);

    ap::AvoidInput in;
    in.v_des = {5.0, 0.0};
    in.rays.fill(ap::kRayRangeM);
    in.neighbors = {{10.0, 0.0, 0.0, 0.0}, {-10.0, 5.0, 0.0, 0.0}, {0.0, -12.0, 0.0, 0.0}};

    double checksum = 0.0;  // uses every result, so the compiler cannot drop the loop
    const auto start = std::chrono::steady_clock::now();
    for (long i = 0; i < iterations; ++i) {
      in.heading = 1e-5 * static_cast<double>(i);            // a different input every step
      in.rays[static_cast<std::size_t>(i) % ap::kNumRays] = 5.0 + static_cast<double>(i % 20);
      const ap::Vec2 c = policy.Correction(in);
      checksum += c.x + c.y;
    }
    const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - start;
    const double us = elapsed.count() / static_cast<double>(iterations);
    std::printf("%ld steps: %.2f us per step (%.0f steps/s), checksum %.6f\n", iterations, us, 1e6 / us, checksum);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
