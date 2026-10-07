// Obstacle-avoidance policy inference in C++17. Standard library only.
//
// C++ port of the three functions the onboard agent needs from src/swarm_agent/avoidance.py:
//   observation()            -> BuildObservation()
//   LearnedPolicy.act()      -> Policy::Act()
//   action_to_correction()   -> ActionToCorrection()
// The Python code stays the reference. tests/test_avoid_policy.cpp checks this port against
// values produced by the Python code (tests/reference_cases.txt).
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace avoid_policy {

// Same constants as avoidance.py. Changing one here without retraining breaks the policy.
inline constexpr std::size_t kNumRays = 24;       // 2-D range readings around the drone
inline constexpr std::size_t kNumNeighbors = 3;   // nearest drones the policy sees
inline constexpr std::size_t kObsDim = 2 + 2 + 2 + kNumRays + 2 + 5 * kNumNeighbors;  // 47
inline constexpr std::size_t kActDim = 2;
inline constexpr std::size_t kMaxLayerWidth = 256;  // upper bound, so Act() needs no heap memory
inline constexpr double kRayRangeM = 25.0;
inline constexpr double kNeighborRangeM = 25.0;
inline constexpr double kVelScaleMps = 10.0;
inline constexpr double kErrScaleM = 20.0;
inline constexpr double kMaxCorrectionMps = 8.0;    // per axis, in the heading frame

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

// Another drone as seen from this one: relative position [m] and velocity [m/s], East-North.
struct Neighbor {
  double rx = 0.0;
  double ry = 0.0;
  double rvx = 0.0;
  double rvy = 0.0;
};

// What one drone knows at one instant. Vectors are East-North; heading is the formation heading [rad].
struct AvoidInput {
  double heading = 0.0;
  Vec2 v_des;                             // velocity asked for by the formation law [m/s]
  Vec2 vel;                               // own velocity [m/s]
  Vec2 slot_err;                          // vector to the formation slot [m]
  std::array<double, kNumRays> rays{};    // distances at heading + k * 15 degrees [m]
  std::optional<Vec2> nearest;            // vector to the closest obstacle point, if one is in range
  std::vector<Neighbor> neighbors;        // any number, any order
};

using Observation = std::array<float, kObsDim>;
using Action = std::array<double, kActDim>;

// Policy input in the formation-heading frame (x forward, y left), scaled to about [-1, 1].
// No heap allocation.
Observation BuildObservation(const AvoidInput& in);

// Policy action in [-1, 1]^2 (heading frame) -> East-North velocity correction [m/s].
Vec2 ActionToCorrection(const Action& action, double heading);

// Deterministic multilayer perceptron: tanh hidden layers, linear output.
class Policy {
 public:
  // Reads a weight file written by export_policy.py. Throws std::runtime_error if the file is
  // missing, truncated, has trailing bytes, or does not describe a kObsDim -> kActDim network.
  static Policy Load(const std::string& path);

  // One forward pass. No heap allocation, no locks: safe to call from a control loop.
  Action Act(const Observation& obs) const;

  // BuildObservation + Act + ActionToCorrection.
  Vec2 Correction(const AvoidInput& in) const;

  std::size_t hidden_layer_count() const { return hidden_.size(); }

 private:
  struct Layer {
    std::size_t rows = 0;      // outputs
    std::size_t cols = 0;      // inputs
    std::vector<double> w;     // rows * cols, row-major
    std::vector<double> b;     // rows
  };

  std::vector<Layer> hidden_;
  Layer out_;
};

}  // namespace avoid_policy
