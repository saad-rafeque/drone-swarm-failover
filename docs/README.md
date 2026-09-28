# Documentation

| Document | Contents |
|---|---|
| [SPECIFICATION.md](SPECIFICATION.md) | The original project specification: goals, hard rules, phases and acceptance criteria. Section and rule numbers quoted in the code and the reports refer to it. |
| [ARCHITECTURE.md](ARCHITECTURE.md) | How the system works, from one drone outwards: agent, heartbeat, leader election, mission, formation, safety layer, obstacle avoidance, simulators and the ground-control app. |
| [RUNBOOK.md](RUNBOOK.md) | Installation from scratch on Ubuntu 24.04 and the command for every test, experiment, figure and document, with troubleshooting. |
| [RESULTS.md](RESULTS.md) | Every result with its numbers and the evidence file it comes from. |
| [DECISIONS.md](DECISIONS.md) | The design decisions that shape the system and the reasons behind them. |
| [KNOWN_ISSUES.md](KNOWN_ISSUES.md) | Known limitations, open work, the road to real drones, and the fixes made in the final check. |
| [FLIGHT_TEST_PLAN.md](FLIGHT_TEST_PLAN.md) | The plan for a first test with one real drone and simulated ones: roles, PX4 safety settings, go/no-go checklist, abort rules and the kill-switch procedure. Not flown. |
| [KAGGLE_GUIDE.md](KAGGLE_GUIDE.md) | Long reinforcement-learning training on a Kaggle GPU, step by step. |
| [HANDOVER.pdf](HANDOVER.pdf) | All of the above, the key figures and every phase report in one printable file. |
| [KAGGLE_GUIDE.pdf](KAGGLE_GUIDE.pdf) | The Kaggle guide as a printable file. |

Related: the phase reports and the evidence behind every number are in [../reports/](../reports/README.md),
and the project history is [../CHANGELOG.md](../CHANGELOG.md).

The PDFs are generated from the Markdown files. After editing a document, rebuild them (Google Chrome
is used to print the PDF):

```bash
PYTHONPATH=src python3 scripts/make_handover_pdf.py
PYTHONPATH=src python3 scripts/make_handover_pdf.py --only kaggle --out docs/KAGGLE_GUIDE.pdf
```
