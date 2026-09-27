# Models

`avoid_policy.npz` is the trained obstacle-avoidance policy. The ground-control app and the agents use it
for the "RL" and "RL + brake" avoiders.

- **Network:** 47 inputs, two hidden layers of 128 (tanh), 2 outputs: a horizontal velocity correction
  in the formation's frame. Stored as plain numpy weights, so no machine-learning library is needed to
  run it (`src/swarm_agent/avoidance.py`).
- **Training:** PPO, 6 million steps on the laptop CPU (38 minutes). This file is the best checkpoint of
  training run `reports/logs/rl/run1/` (`best_policy.npz`).
- **Results:** [docs/RESULTS.md](../docs/RESULTS.md): 90 unseen courses and a real route across
  Islamabad.
- **Replacing it:** copy a newer `best_policy.npz` over this file, for example from a Kaggle run
  ([docs/KAGGLE_GUIDE.md](../docs/KAGGLE_GUIDE.md)). The app and the agents load it automatically.
