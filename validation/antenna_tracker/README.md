# Antenna Tracker Validation Evidence

This directory is the versioned, machine-readable verification contract for the PX4 antenna tracker. It replaces checklist-by-file-existence and unstructured console transcript claims.

## Files

- `test_matrix.yaml` — canonical test IDs, required gates, prerequisites, and acceptance criteria.
- `evidence.schema.yaml` — required fields for a single test-run manifest.
- `legacy/README.md` — limitations of old transcripts retained only as historical context.

Raw ULogs, large videos, photos, and binary firmware artifacts should not be committed here. Store them in the project artifact location and record an immutable URI/path plus SHA-256 checksum in the evidence manifest.

## Evidence manifest layout

Store each evidence manifest outside generated build/log directories, for example:

```text
validation/antenna_tracker/runs/<gate>/<test-id>/<run-id>.yaml
```

A run is valid only when it:

1. conforms to `evidence.schema.yaml`;
2. references a test ID in `test_matrix.yaml`;
3. records firmware and setup provenance;
4. includes raw evidence locations/checksums;
5. has a `pass`, `fail`, or `blocked` verdict.

## Status vocabulary

- `planned`: no implementation claim.
- `implemented`: source exists but runtime behavior is not verified.
- `verified-sitl`: canonical tracker SITL case passed.
- `verified-bench`: controlled bench case passed on intended hardware.
- `verified-hardware`: deployed sensor/telemetry/mechanical setup passed.
- `field-ready`: repeated outdoor and failure cases passed.

Do not upgrade a state based only on a local build or manually edited markdown checkbox.