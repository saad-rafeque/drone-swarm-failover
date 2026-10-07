# avoid_policy: C++ inference for the obstacle-avoidance policy

A C++17 port of the onboard half of [`src/swarm_agent/avoidance.py`](../../src/swarm_agent/avoidance.py).
It builds the 47-value observation, runs the 47-128-128-2 tanh network and rotates the action into an
East-North velocity correction. It uses the standard library only, so it can be compiled into a
companion-computer process without Python or a machine-learning runtime.

**Status.** Checked against the Python code on reference inputs (below). It has not run inside a ROS 2
node or on a real drone; the Python agent is still what every result in this repository was produced with.

## Files

| File | Contents |
|---|---|
| `avoid_policy.hpp`, `avoid_policy.cpp` | The library: `BuildObservation()`, `Policy::Load()`, `Policy::Act()`, `Policy::Correction()`, `ActionToCorrection()` |
| `avoid_policy.bin` | The trained weights of `models/avoid_policy.npz` in the format described below |
| `export_policy.py` | Writes `avoid_policy.bin` and the reference cases from the `.npz`; `--check` verifies the `.bin` is current |
| `tests/test_avoid_policy.cpp` | The test program (no test framework needed) |
| `tests/reference_cases.txt` | 100 seeded random inputs with the observation, action and correction the Python code gives |
| `bench_avoid_policy.cpp` | Times one avoidance step on the machine it runs on |
| `CMakeLists.txt` | Builds the library, the benchmark and the test |

## Build and test

Needs CMake 3.16 or newer and a C++17 compiler. From the repository root:

```bash
cmake -S cpp/avoid_policy -B build/avoid_policy
cmake --build build/avoid_policy
ctest --test-dir build/avoid_policy --output-on-failure
build/avoid_policy/bench_avoid_policy cpp/avoid_policy/avoid_policy.bin
```

## Use

```cpp
#include "avoid_policy.hpp"

const auto policy = avoid_policy::Policy::Load("avoid_policy.bin");   // once, at start-up

avoid_policy::AvoidInput in;
in.heading = formation_heading;          // rad
in.v_des = {5.0, 0.0};                   // m/s, East-North
in.rays = lidar_ranges;                  // 24 distances, heading + k * 15 degrees
in.neighbors = {{10.0, 0.0, 0.0, 0.0}};  // relative position and velocity of other drones

const avoid_policy::Vec2 dv = policy.Correction(in);   // add to v_des
```

## How it is checked

- **Against Python.** For 100 seeded random inputs the test compares the C++ observation, action and
  end-to-end correction with the values `avoidance.py` produced. The inputs cover the clamped slot error,
  rays beyond the 25 m range, no obstacle in range, and 0 to 5 neighbours with some out of range. Limits:
  1e-6 for the single-precision observation, 1e-9 for the action. Measured with GCC 13.3 and Clang 18.1 on
  x86-64: observation identical, action within 4e-16.
- **Neighbour selection.** The three nearest drones in range, nearest first, equal distances in input order.
- **Bad weight files.** A missing file, a wrong magic number, a truncated file, trailing bytes and a layer
  that does not fit the observation are each rejected with an exception.
- **Compiler checks.** The library builds with `-Wall -Wextra -Wpedantic -Wconversion -Werror`. The test
  also passes under AddressSanitizer and UndefinedBehaviorSanitizer.

## Design notes

- **No heap memory in the control loop.** `BuildObservation()` and `Policy::Act()` use fixed-size arrays
  only, so their run time does not depend on the allocator. Layers are limited to 256 neurons for this.
- **Same arithmetic as training.** The observation is single precision and the weights are double
  precision, as in the Python code. `-ffast-math` is not used, because it would let the compiler reorder
  the sums and break the comparison with Python.
- **Weight file.** Little-endian: 8 bytes `AVPOL001`, a `uint32` number of hidden layers, then for every
  layer (hidden layers first, output layer last) `uint32 rows`, `uint32 cols`, `rows * cols` `float64`
  weights in row-major order and `rows` `float64` biases. `Policy::Load()` refuses to run on a big-endian
  host.

## After retraining

```bash
python cpp/avoid_policy/export_policy.py          # rewrites avoid_policy.bin and the reference cases
python cpp/avoid_policy/export_policy.py --check  # exit code 1 if avoid_policy.bin is out of date
```

Commit both generated files with the new `.npz`.

## Limits

- This is a library, not a ROS 2 node. Wrapping it in an `rclcpp` node is the next step.
- Only the learned policy is ported. The stopping-distance brake (`Shielded`) and the potential-field
  avoider exist in Python only, and the results in [docs/RESULTS.md](../../docs/RESULTS.md) use the brake.
- The benchmark reports the machine it runs on. No timing is claimed for flight hardware.
