## static_v2 results (KPI summary)

This table summarizes the absolute pose error (APE) statistics of Cartographer localization (`/tracked_pose`) against ground truth.

| run bag | RMSE (m) | mean (m) | median (m) | max (m) | std (m) |
|---|---:|---:|---:|---:|---:|
| `loc_v2_odom` | 0.03295 | 0.03051 | 0.03093 | 0.06127 | 0.01245 |
| `loc_v2_noisy_tf` | 0.33258 | 0.26311 | 0.18674 | 1.00114 | 0.20342 |
| `loc_v2_fused_ekf_v2` | 0.08115 | 0.06654 | 0.05416 | 0.23572 | 0.04646 |

### Takeaway

Under slip/degraded wheel odometry, the KISS-ICP + EKF fusion pipeline improves Cartographer localization from ~0.33 m RMSE to ~0.08 m RMSE (still worse than the clean-odom baseline ~0.03 m RMSE).

