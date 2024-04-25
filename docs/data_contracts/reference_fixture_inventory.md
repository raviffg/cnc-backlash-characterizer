# Reference fixture inventory

## Scope
Data handling reference for CNC leadscrew backlash characterization and G-code motion simulation.

## Relevant sources
- Input/measurement source: inspect `scripts/backlash_fit.py` and the repository's existing data files.
- Consumer/algorithm: `src/gcode_stepper_sim.cpp`.
- Domain concerns: axis reversal, direction state, encoder synchronization, arc interpolation, feed sensitivity, repeatability.

## Contract requirements
- Preserve source values and original units.
- State column/field names, types, units, valid ranges, and whether a field is optional.
- Define timestamp origin, sampling assumptions, and ordering requirements where applicable.
- Distinguish missing, invalid, saturated, and intentionally masked values.
- Keep calibration identifiers and acquisition metadata alongside measurements.
- Reject malformed records with an actionable diagnostic; do not silently invent defaults.

## Provenance
For every derived dataset, record its source, transformation script, parameters, timestamp, and checksum. Mark simulated or synthetic data clearly and never present it as a measured capture.

## Review status
Template for project-specific completion. Confirm exact field names and constraints against the actual parser and checked-in fixtures before publishing a formal schema.
