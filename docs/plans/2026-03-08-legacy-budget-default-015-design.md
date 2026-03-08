# Legacy Budget Default 0.15 Design Note

## Decision

Change the shared legacy `vision_inject.max_age_sec` default from `0.20s` to `0.15s`.

## Why

The 2026-03-08 adapter replay sweep showed:
- `0.20s` preserves the most late fusion, but adds excessive wait in raw-only and fallback-heavy cases.
- `0.08s`, `0.10s`, and `0.12s` reduce wait further, but collapse late-fused and pressure-path fusion too aggressively.
- `0.15s` is the best tested single default compromise: it materially reduces wait cost while preserving meaningful fusion benefit in legacy mode.

## Scope

This change only updates the shared legacy default and aligned code fallbacks.
It does not change adapter state-machine semantics, TF/camera handling, or calibrated fusion behavior.

## Follow-up

Do not keep iterating on one global default.
The next line of work is a single-source mission-specific budget parameter so missions can choose between latency-priority and fusion-priority budgets without reintroducing multiple independent timing definitions.
