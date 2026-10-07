#include "avoid_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace avoid_policy {
namespace {

constexpr char kMagic[8] = {'A', 'V', 'P', 'O', 'L', '0', '0', '1'};

// Rotates (x, y) by the angle whose cosine and sine are c and s.
Vec2 Rotate(double x, double y, double c, double s) { return {x * c - y * s, x * s + y * c}; }

bool HostIsLittleEndian() {
  const std::uint16_t one = 1;
  unsigned char first = 0;
  std::memcpy(&first, &one, 1);
  return first == 1;
}

template <typename T>
T ReadValue(std::ifstream& f, const std::string& path) {
  T value{};
  f.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!f) throw std::runtime_error("avoid_policy: truncated weight file: " + path);
  return value;
}

std::vector<double> ReadDoubles(std::ifstream& f, std::size_t count, const std::string& path) {
  std::vector<double> values(count);
  f.read(reinterpret_cast<char*>(values.data()),
         static_cast<std::streamsize>(count * sizeof(double)));
  if (!f) throw std::runtime_error("avoid_policy: truncated weight file: " + path);
  return values;
}

}  // namespace

Observation BuildObservation(const AvoidInput& in) {
  // Rotating by -heading takes East-North vectors into the heading frame.
  const double c = std::cos(-in.heading);
  const double s = std::sin(-in.heading);
  Observation o{};  // zero-filled: absent obstacle and absent neighbours stay 0

  const Vec2 v_des = Rotate(in.v_des.x, in.v_des.y, c, s);
  o[0] = static_cast<float>(v_des.x / kVelScaleMps);
  o[1] = static_cast<float>(v_des.y / kVelScaleMps);

  const Vec2 vel = Rotate(in.vel.x, in.vel.y, c, s);
  o[2] = static_cast<float>(vel.x / kVelScaleMps);
  o[3] = static_cast<float>(vel.y / kVelScaleMps);

  const Vec2 err = Rotate(in.slot_err.x, in.slot_err.y, c, s);
  o[4] = static_cast<float>(std::clamp(err.x / kErrScaleM, -1.5, 1.5));
  o[5] = static_cast<float>(std::clamp(err.y / kErrScaleM, -1.5, 1.5));

  // The Python code divides in single precision here, so this does too.
  for (std::size_t i = 0; i < kNumRays; ++i) {
    const float r = static_cast<float>(in.rays[i]) / static_cast<float>(kRayRangeM);
    o[6 + i] = std::clamp(r, 0.0f, 1.0f);
  }

  if (in.nearest) {
    const Vec2 n = Rotate(in.nearest->x, in.nearest->y, c, s);
    o[6 + kNumRays] = static_cast<float>(n.x / kRayRangeM);
    o[7 + kNumRays] = static_cast<float>(n.y / kRayRangeM);
  }

  // The kNumNeighbors nearest drones inside the range, nearest first. Insertion into a fixed array:
  // no heap memory, and equal distances keep their input order, as Python's sorted() does.
  const auto dist2 = [](const Neighbor& n) { return n.rx * n.rx + n.ry * n.ry; };
  std::array<const Neighbor*, kNumNeighbors> nearest{};
  std::size_t count = 0;
  for (const Neighbor& n : in.neighbors) {
    const double d2 = dist2(n);
    if (d2 >= kNeighborRangeM * kNeighborRangeM) continue;
    std::size_t pos = count;
    while (pos > 0 && d2 < dist2(*nearest[pos - 1])) --pos;
    if (pos >= kNumNeighbors) continue;  // farther than the ones already kept
    for (std::size_t k = std::min(count, kNumNeighbors - 1); k > pos; --k) nearest[k] = nearest[k - 1];
    nearest[pos] = &n;
    if (count < kNumNeighbors) ++count;
  }

  const std::size_t base = 8 + kNumRays;
  for (std::size_t k = 0; k < count; ++k) {
    const std::size_t j = base + 5 * k;
    const Vec2 p = Rotate(nearest[k]->rx, nearest[k]->ry, c, s);
    const Vec2 v = Rotate(nearest[k]->rvx, nearest[k]->rvy, c, s);
    o[j] = static_cast<float>(p.x / kNeighborRangeM);
    o[j + 1] = static_cast<float>(p.y / kNeighborRangeM);
    o[j + 2] = static_cast<float>(v.x / kVelScaleMps);
    o[j + 3] = static_cast<float>(v.y / kVelScaleMps);
    o[j + 4] = 1.0f;  // "this neighbour slot is filled"
  }
  return o;
}

Vec2 ActionToCorrection(const Action& action, double heading) {
  const double ax = std::clamp(action[0], -1.0, 1.0) * kMaxCorrectionMps;
  const double ay = std::clamp(action[1], -1.0, 1.0) * kMaxCorrectionMps;
  return Rotate(ax, ay, std::cos(heading), std::sin(heading));
}

Policy Policy::Load(const std::string& path) {
  if (!HostIsLittleEndian()) {
    throw std::runtime_error("avoid_policy: weight files are little-endian; this host is not");
  }
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("avoid_policy: cannot open weight file: " + path);

  char magic[sizeof(kMagic)] = {};
  f.read(magic, sizeof(magic));
  if (!f || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    throw std::runtime_error("avoid_policy: not a policy weight file: " + path);
  }

  const auto read_layer = [&](std::size_t expected_cols) {
    Layer layer;
    layer.rows = ReadValue<std::uint32_t>(f, path);
    layer.cols = ReadValue<std::uint32_t>(f, path);
    if (layer.cols != expected_cols || layer.rows == 0 || layer.rows > kMaxLayerWidth) {
      throw std::runtime_error("avoid_policy: unexpected layer shape in " + path);
    }
    layer.w = ReadDoubles(f, layer.rows * layer.cols, path);
    layer.b = ReadDoubles(f, layer.rows, path);
    return layer;
  };

  Policy policy;
  const std::uint32_t hidden_count = ReadValue<std::uint32_t>(f, path);
  if (hidden_count == 0 || hidden_count > 16) {
    throw std::runtime_error("avoid_policy: unexpected layer count in " + path);
  }
  std::size_t width = kObsDim;
  for (std::uint32_t k = 0; k < hidden_count; ++k) {
    policy.hidden_.push_back(read_layer(width));
    width = policy.hidden_.back().rows;
  }
  policy.out_ = read_layer(width);
  if (policy.out_.rows != kActDim) {
    throw std::runtime_error("avoid_policy: output layer must have 2 rows in " + path);
  }
  if (f.peek() != std::ifstream::traits_type::eof()) {
    throw std::runtime_error("avoid_policy: trailing bytes in weight file: " + path);
  }
  return policy;
}

Action Policy::Act(const Observation& obs) const {
  // Two fixed-size buffers on the stack, swapped after every layer.
  std::array<double, kMaxLayerWidth> a{};
  std::array<double, kMaxLayerWidth> b{};
  static_assert(kObsDim <= kMaxLayerWidth, "the observation must fit the buffer");
  for (std::size_t i = 0; i < kObsDim; ++i) a[i] = static_cast<double>(obs[i]);

  double* x = a.data();
  double* y = b.data();
  for (const Layer& layer : hidden_) {
    const double* w = layer.w.data();
    for (std::size_t r = 0; r < layer.rows; ++r, w += layer.cols) {
      double sum = layer.b[r];
      for (std::size_t col = 0; col < layer.cols; ++col) sum += w[col] * x[col];
      y[r] = std::tanh(sum);
    }
    std::swap(x, y);
  }

  Action action{};
  const double* w = out_.w.data();
  for (std::size_t r = 0; r < kActDim; ++r, w += out_.cols) {
    double sum = out_.b[r];
    for (std::size_t col = 0; col < out_.cols; ++col) sum += w[col] * x[col];
    action[r] = sum;  // linear output; ActionToCorrection() clips it
  }
  return action;
}

Vec2 Policy::Correction(const AvoidInput& in) const {
  return ActionToCorrection(Act(BuildObservation(in)), in.heading);
}

}  // namespace avoid_policy
