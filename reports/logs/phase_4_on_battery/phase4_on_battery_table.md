| Fault | Trials | New leader median / worst (s) | Limit (s) | Formation < 2 m median / worst (s) | Recovered within 15 s | Closest pair (m) | Goal reached | Other |
|---|---|---|---|---|---|---|---|---|
| F1 | 3 | 2.19 / 2.23 | 3.0 / 4.0 | 7.54 / 8.11 | 3/3 | 7.41 | 2/3 |  |
| F2 | 2 | 1.84 / 1.94 | 3.0 / 4.0 | 7.92 / 8.51 | 2/2 | 8.00 | 2/2 |  |
| F3 | 2 | 0.57 / 0.80 | 1.0 | 6.88 / 7.78 | 2/2 | 7.17 | 2/2 | old leader landed at home: 2/2 |
| F4 | 2 | no change | no change | 7.90 / 8.45 | 2/2 | 7.96 | 2/2 | leader changed: 0/2 |
| F5 | 2 | 0.93 / 1.55 | 3.0 (after heal) | 16.89 / 17.08 | 0/2 | 6.11 | 2/2 |  |

| Trial | Leader before | Target | Fault at (s) | New leader (s) | New leader id | Formation < 2 m (s) | Max RMS after (m) | Closest pair (m) | Goal |
|---|---|---|---|---|---|---|---|---|---|
| F1_t1 | 1 | 1 | 106.6 | 2.23 | 2 | 7.54 | 7.35 | 7.99 | yes |
| F1_t2 | 1 | 1 | 207.8 | 2.10 | 2 | 8.11 | 7.50 | 7.41 | yes |
| F1_t3 | 1 | 1 | 111.0 | 2.19 | 2 | 7.43 | 19.66 | 8.45 | no |
| F2_t1 | 1 | 1 | 98.7 | 1.74 | 2 | 7.32 | 6.99 | 8.00 | yes |
| F2_t2 | 1 | 1 | 201.1 | 1.94 | 2 | 8.51 | 7.51 | 8.17 | yes |
| F2_t3 | error: JSONDecodeError('Expecting value: line 1 column 1 (char 0)') | | | | | | | | |
| F3_t1 | 1 | 1 | 100.8 | 0.35 | 2 | 5.97 | 6.93 | 7.84 | yes |
| F3_t2 | 1 | 1 | 194.5 | 0.80 | 2 | 7.78 | 10.81 | 7.17 | yes |
| F4_t1 | 1 | 3 | 102.5 | - | - | 8.45 | 22.25 | 7.96 | yes |
| F4_t2 | 1 | 2 | 196.1 | - | - | 7.36 | 6.99 | 8.24 | yes |
| F5_t1 | 1 | - | 99.9 | 1.55 | - | 17.08 | 29.61 | 6.52 | yes |
| F5_t2 | 1 | - | 196.7 | 0.30 | - | 16.71 | 29.25 | 6.11 | yes |
