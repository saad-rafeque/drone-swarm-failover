# Kaggle

`train_swarm_rl.ipynb` is the notebook for long reinforcement-learning training of the
obstacle-avoidance policy on a Kaggle GPU.

The code it needs is packed by `python3 scripts/make_kaggle_bundle.py` into `kaggle/swarm-rl-code.zip`
and uploaded to Kaggle as a dataset. The zip is built on demand and is not stored in the repository; the
script skips `*.local.*` files and refuses to build if any packed file contains a map key.

Step-by-step guide: [docs/KAGGLE_GUIDE.md](../docs/KAGGLE_GUIDE.md) (also `docs/KAGGLE_GUIDE.pdf`).
Status: the kit is ready and was checked on the laptop with tiny settings; it has not been run on Kaggle
yet.
