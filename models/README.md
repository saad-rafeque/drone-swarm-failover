# Models

`avoid_policy.npz` is the trained obstacle-avoidance policy. The ground-control app and the agents use it
for the "RL" and "RL + brake" avoiders.

- **Network:** 47 inputs, two hidden layers of 128 (tanh), 2 outputs: a horizontal velocity correction
  in the formation's frame. Stored as plain numpy weights, so no machine-learning library is needed to
  run it (`src/swarm_agent/avoidance.py`).
- **Training:** PPO on a Kaggle T4 GPU, seed 2: 5.84 billion steps in 10.5 hours (29 September 2026). This file
  is that run's best checkpoint on the validation courses (`reports/logs/rl/kaggle_seed2/best_policy.npz`,
  learning curve `reports/rl_training_kaggle_seed2.png`).
- **`avoid_policy_run1.npz`:** the earlier policy, 6 million steps on the laptop CPU (38 minutes,
  `reports/logs/rl/run1/`), kept for comparison. Copy it over `avoid_policy.npz` to go back.
- **Results:** [docs/RESULTS.md](../docs/RESULTS.md): 90 unseen courses and a real route across
  Islamabad.
- **Replacing it:** copy a newer `best_policy.npz` over this file after the check in
  [docs/KAGGLE_GUIDE.md](../docs/KAGGLE_GUIDE.md), section 8. The app and the agents load it automatically.
