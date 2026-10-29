# Four-degree-of-freedom experiment patches

These patches apply to the workspace snapshot current when they were produced.
They contain only the relocalization manager's optional four-degree-of-freedom
registration experiment, its helper header, its synthetic test, and its test
build dependency. The roll/pitch/yaw diagnostic logging is retained by removal.
They do not modify loam, Point-LIO, simulation profiles, or safety checks.

From the repository root, verify and remove after the experiment is no longer
selected:

```bash
git apply --check tools/navigation_benchmark/results/20261028_followup/validation/remove_4d_experiment.patch
git apply tools/navigation_benchmark/results/20261028_followup/validation/remove_4d_experiment.patch
```

To restore the exact experimental implementation and test (provided the touched
files have not changed in conflicting ways):

```bash
git apply --check tools/navigation_benchmark/results/20261028_followup/validation/restore_4d_experiment.patch
git apply tools/navigation_benchmark/results/20261028_followup/validation/restore_4d_experiment.patch
```

Restoring code alone does not enable the option; the experimental profile must
explicitly set `gravity_aligned_registration: true` for a validated aligned
map and odometry frame. Rebuild and run the relevant tests before using it.
Round-trip patch application was checked in a temporary copy; no workspace source
was changed to generate these artifacts.
