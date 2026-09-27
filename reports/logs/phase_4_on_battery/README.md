First Phase 4 attempt, 27 September 2026, 12:31-13:57. NOT used for the Phase 4 results.

The laptop was running on battery in the power-saver profile (CPU clock about 1.7 GHz instead of up to
4.1 GHz). With 10 PX4 drones the CPU was saturated for the whole run (load about 60, CPU 99.9 %; Phase 3
ran at load about 17 and 67 % CPU on the charger), and the battery ran out at 13:57 (the system log ends
without a shutdown). Trial F1_t3 ended with the harness seeing no drone messages for 18 s (the whole
machine stalled); F2_t3 was cut off by the power loss.

The 10 completed trials (rounds 1 and 2 of F1-F5) are kept for transparency. They passed every
takeover and separation check; F5's formation recovery after the heal took about 17 s in both rounds
(limit 15 s). Summary: phase4_on_battery_table.json / .md (made with scripts/phase4_metrics.py).
Since then, scripts/phase4_runs.sh waits for the charger and a non-power-saver profile before every
trial, and every run_summary.json records the power state.
