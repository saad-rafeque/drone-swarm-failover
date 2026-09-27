"""Safety checks on helper scripts: the Kaggle bundle never packs secrets, and the GPU training pipeline
keeps its validation courses (used to choose the best policy) apart from the test courses it reports on."""
from __future__ import annotations

import importlib.util
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_kaggle_bundle_skips_local_key_files_and_refuses_secrets(tmp_path, monkeypatch):
    mod = load("make_kaggle_bundle")
    files = mod.packed_files()
    assert files and not any(".local." in f.name for f in files)
    assert all("__pycache__" not in f.parts for f in files)
    # a packed file that contains a key value from the local keys file stops the build
    keys = tmp_path / "map_keys.local.yaml"
    keys.write_text('mapbox_token: "pk.test-value-that-is-long-enough"\n')
    leak = tmp_path / "leak.txt"
    leak.write_text("token=pk.test-value-that-is-long-enough")
    monkeypatch.setattr(mod, "KEYS_FILE", keys)
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    monkeypatch.setattr(mod, "INCLUDE", ["leak.txt"])
    try:
        mod.main()
    except SystemExit as exc:
        assert "refusing to pack leak.txt" in str(exc)
    else:
        raise AssertionError("the bundle packed a file containing a key")
    assert not (tmp_path / "kaggle" / "swarm-rl-code.zip").exists()


def test_existing_kaggle_zip_has_no_local_files():
    z = ROOT / "kaggle" / "swarm-rl-code.zip"
    if z.exists():
        assert not any(".local." in n for n in zipfile.ZipFile(z).namelist())


def test_validation_and_test_courses_do_not_overlap():
    pools = load("build_pools")
    rl_eval_src = (ROOT / "scripts" / "rl_eval.py").read_text()
    assert "EVAL_SEED0 = TRAIN_SEED_MAX + 100" in rl_eval_src
    assert pools.EVAL_SEED0 == pools.TRAIN_SEED_MAX + 100 and pools.VAL_SEED0 == pools.TRAIN_SEED_MAX
    # the default 30 validation courses per level use seeds TRAIN_SEED_MAX + 0..29, the test ones + 100..129
    assert pools.VAL_SEED0 + 30 <= pools.EVAL_SEED0
    nb = (ROOT / "kaggle" / "train_swarm_rl.ipynb").read_text()
    assert "pools/val.npz" in nb and "'--eval-pool', '/kaggle/working/pools/heldout.npz'" not in nb
