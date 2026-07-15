# Final holdout registry

This registry freezes evaluation data before it is converted, inspected for
estimator behaviour, or used for parameter tuning. A holdout may be evaluated
only once, after the Phase 0--5 implementation and tuning decisions are
frozen. Any subsequent tuning invalidates that evaluation and requires a new,
previously unused holdout.

## UrbanNav Tokyo Shinjuku

- Registered: 2026-07-15 (Asia/Tokyo)
- Role: final blind urban holdout
- Tuning use at registration: none
- Evaluation at registration: none
- Source directory: `.data/urbannav_tokyo/Shinjuku`

Only filenames, byte sizes, and SHA-256 values were collected during
registration; file contents and reference-derived metrics were not inspected.

| File | Bytes | SHA-256 |
|---|---:|---|
| `base.nav` | 6,916,149 | `c58bd7ecaf7c7a0766fa9bc23ba7b0666172c28c95a13c35ac973f88b2645b35` |
| `base_trimble.obs` | 18,830,551 | `34a851c54e07b8cac592d37539c85e19d02e1a6b29839a707ab69dba9692c99e` |
| `imu.csv` | 11,946,246 | `0ab3dd6aed850f44d44910d483b3f082a95ae20076a2e78cc62a15d1ce0b2ba9` |
| `reference.csv` | 5,101,472 | `794baaec530f34097cd1f77633902bb2195ce30cdec278945eb5ac911ed65d79` |
| `rover_trimble.obs` | 95,755,323 | `be239dbd20d9d69a0dbc696f1a4b00d75cd3639354d498eec91f08fa65d000eb` |
| `rover_ublox.obs` | 40,539,929 | `dece3e6fbbe47e78705570d0e99796460d6d34f75a5dbbf20664a5446def7c3f` |

