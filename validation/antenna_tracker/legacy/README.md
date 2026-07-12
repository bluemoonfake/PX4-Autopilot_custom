# Legacy Tracker Test Material

The following files are retained as historical notes only:

- repository-root `test_case`;
- repository-root `test_case_2`;
- `logs/verification/phase1_sitl.txt` from the prior generated-log directory;
- previous overview, TODO, walkthrough, and implementation notes migrated into the canonical documentation tree.

They do **not** satisfy the current evidence contract because they lack a complete manifest and include invalid or ambiguous setup conditions, including:

- use of `make px4_sitl gz_x500`, which boots the multicopter autostart instead of validating tracker airframe 4099;
- paths tied to an old checkout location;
- target sender output showing `udp:127.0.0.1:14550` while the intended tracker target ingress is a distinct configured receiver port;
- results where `tracker_target_position` was never published;
- claims based on fake target parameters rather than external MAVLink ingress.

Do not delete historical material until it has been reviewed for any facts worth preserving, but do not mark a feature verified from it. Re-run the applicable test ID in `../test_matrix.yaml` and create an evidence manifest instead.