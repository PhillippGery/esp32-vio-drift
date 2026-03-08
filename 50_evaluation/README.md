# 50_evaluation — Results & Analysis

Owned by **Jack** (Evaluation & Visualization).

| Folder | Contents |
|---|---|
| `test_results/` | Drift comparison tables (CSV/XLSX), raw test run data |
| `plots/` | Drift trajectory plots, classifier accuracy charts (PNG/SVG) |

## Key Metrics

| Metric | Target | Description |
|---|---|---|
| Position drift | < 5% per metre | EKF vs. ground truth (tape measure) |
| Orientation drift | < 2° per 10 s | Yaw accumulation at rest |
| Classifier accuracy | > 90% | TinyML motion classifier (INT8) |
| IMU-only drift baseline | — | Node 2 deadreckoning for comparison |

## Test Protocol

See `test_results/test_protocol_v1.md` for the full evaluation procedure,
including the straight-line walk, rotation, and figure-8 tests.
