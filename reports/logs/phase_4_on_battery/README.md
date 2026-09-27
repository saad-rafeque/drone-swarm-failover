# Phase 4, first attempt on battery power (27 September 2026)

**Not used for the Phase 4 results.** These trials are kept for transparency.

## What happened
From 12:31 to 13:57 the laptop ran on battery in the power-saver profile, with the CPU clock at about
1.7 GHz instead of up to 4.1 GHz. With 10 PX4 drones the CPU was saturated for the whole run (load
average about 60, CPU 99.9 %; Phase 3 ran at a load of about 17 and 67 % CPU on the charger), and the
battery ran out at 13:57 (the system log ends without a shutdown). Trial `F1_t3` ended when the test
harness received no drone messages for 18 s because the whole machine had stalled; `F2_t3` was cut off
by the power loss.

## Results
The 10 completed trials (rounds 1 and 2 of F1 to F5) passed every takeover and separation check. In
both F5 trials the formation took about 17 s to re-form after the heal (limit 15 s).

- Tables: `phase4_on_battery_table.json` and `phase4_on_battery_table.md`, made with
  `scripts/phase4_metrics.py`.
- Figure: `reports/phase4_on_battery.png`, made with `scripts/plot_phase4.py`.
- Report: `reports/PHASE_4.md`.

## What changed afterwards
`scripts/phase4_runs.sh` now waits for the charger and a profile other than power saver before every
trial, and every `run_summary.json` records the power state.
