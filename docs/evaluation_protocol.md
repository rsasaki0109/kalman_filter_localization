# Localization evaluation protocol

## Dataset split and provenance

Freeze input hashes, conversion commands, reference hashes, profile YAML, repository state, and ROS
environment in `manifest.json`. Profile sweeps are allowed only on the tuning split. A holdout run
must specify `--split-role holdout` and exactly one frozen final profile; evaluating a holdout makes
it spent. Registered blind datasets are listed in [`holdout_registry.md`](holdout_registry.md).

## Association and alignment

The default policy uses sensor timestamps, linear reference interpolation, 0.2 s maximum reference
gap, zero time offset, and no alignment. Translation alignment is allowed only for a segment whose
gate explicitly says so and uses one median translation for the whole evaluated segment. Store the
alignment choice, interpolation tolerance, time offset, interval, and named segments in the
manifest. Report missing ratio; unmatched samples are never silently discarded.

## Metrics and gates

Report 3D, horizontal, vertical, and yaw APE; time- and distance-indexed RPE; and, for every GNSS
outage, endpoint drift, post-reacquisition overshoot, and settling time. Simulation additionally
reports state NEES and measurement NIS with chi-square 95% coverage. Input topic counts must match
rosbag metadata, output timestamps must be monotonic, covariance must remain finite/PSD, and all
measurement rejects must carry a reason.

The frozen regression thresholds are:

- open-sky and urban-continuous position/yaw no worse than 105% of Phase 0;
- Odaiba 81.8 s outage no worse than 8.1117 m for auto scale or 6.3358 m for fixed scale;
- three repeats agree within the declared floating-point tolerance;
- systematic NEES/NIS overconfidence fails even when trajectory accuracy passes.

Pull-request CI runs unit, Monte Carlo, parameter-schema, and deterministic short-bag replay tests.
Full registered bags run in the scheduled/manual workflow, which retains manifests, comparisons,
and logs. The detailed results and commands are in
[`research_evaluation.md`](research_evaluation.md).

